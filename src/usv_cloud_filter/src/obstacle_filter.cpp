// Copyright 2026 jkjkzhang
// SPDX-License-Identifier: MIT
//
// Water-surface obstacle filter (M3).
//
// Consumes Point-LIO's registered cloud (frame camera_init, the LIO world
// frame) and produces TWO clean obstacle clouds for the Nav2 costmaps:
//
//   /usv/structure_cloud   strict band [z_min, z_max]   (default 0.55..3.0)
//                          -> the structure the boat must avoid. Dropping the
//                          z<0.55 slice removes water-surface reflections /
//                          spray that the wide band lets through and that made
//                          the costmap noisy. Shore / buoys have vertical
//                          extent above z=0.55, so their 2D footprint survives.
//   /usv/costmap_cloud     wide band [z_min, z_max]     (default 0.05..3.0)
//                          kept for comparison/rollback.
//
// Height rejection is done after conversion to base_link, where water is z ~= 0.
//
// Pipeline (all PCL, offline, no downloads):
//   1. finite-point removal        (Gazebo emits inf for no-return rays)
//   2. transform camera_init -> output_frame (default wamv/wamv/base_link)
//   3. height-band crop in the BOAT frame (drops water despite LIO z drift)
//   4. distance crop in output frame (local to the moving boat)
//   5. voxel-grid downsample       (eases the costmap + outlier step)
//   6. radius outlier removal      (drops isolated spray / wave-crest specks)
//
// The final transform is the KEY part for Nav2: the obstacle layer treats the
// observation's frame origin as the sensor origin for range filtering and
// raytrace-clearing. If we published in camera_init (== costmap global frame)
// that origin would be the MAP origin, so obstacle_max_range and clearing
// would be relative to where the boat STARTED, not where it IS. Publishing in
// the boat's base_link makes the sensor origin the boat -> correct local
// avoidance regardless of distance from the map origin.
//
//   in : /cloud_registered      (sensor_msgs/PointCloud2, frame camera_init)

#include <cmath>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct MapVoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const MapVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct MapVoxelKeyHash
{
  std::size_t operator()(const MapVoxelKey & key) const
  {
    const auto hx = std::hash<int>{}(key.x);
    const auto hy = std::hash<int>{}(key.y);
    const auto hz = std::hash<int>{}(key.z);
    return hx ^ (hy + 0x9e3779b9U + (hx << 6U) + (hx >> 2U)) ^
           (hz + 0x9e3779b9U + (hy << 6U) + (hy >> 2U));
  }
};

class ObstacleFilter : public rclcpp::Node
{
public:
  ObstacleFilter()
  : Node("obstacle_filter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cloud_registered");
    // Direct sensor cloud used only by the independent emergency stop. It is
    // intentionally not derived from Point-LIO's accumulated map: if mapping
    // lags or its frame drifts, the boat must still stop for a nearby shore.
    raw_input_topic_ = declare_parameter<std::string>(
      "raw_input_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points");
    safety_output_topic_ = declare_parameter<std::string>(
      "safety_output_topic", "/usv/safety_cloud");
    raw_obstacle_output_topic_ = declare_parameter<std::string>(
      "raw_obstacle_output_topic", "/usv/raw_obstacle_cloud");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/usv/costmap_cloud");
    // Frame the filtered clouds are transformed into before publishing. Must be
    // the boat base frame so Nav2's obstacle layer uses the boat as sensor
    // origin (see file header). Use "camera_init" to disable the transform.
    output_frame_ = declare_parameter<std::string>(
      "output_frame", "wamv/wamv/base_link");

    // Wide band (legacy): keeps the original behavior on /usv/costmap_cloud.
    z_min_ = declare_parameter<double>("z_min", 0.05);
    z_max_ = declare_parameter<double>("z_max", 3.0);

    // Strict band: structure the boat must avoid.  The crop is applied after
    // conversion to base_link, where z ~= 0 is the water surface.  This avoids
    // depending on Point-LIO's arbitrary/drifting camera_init height.
    structure_output_topic_ = declare_parameter<std::string>(
      "structure_output_topic", "/usv/structure_cloud");
    structure_map_output_topic_ = declare_parameter<std::string>(
      "structure_map_output_topic", "/usv/structure_map_cloud");
    world_output_frame_ = declare_parameter<std::string>(
      "world_output_frame", "camera_init");
    structure_map_accumulate_ = declare_parameter<bool>(
      "structure_map_accumulate", true);
    structure_map_voxel_size_ = declare_parameter<double>(
      "structure_map_voxel_size", 0.5);
    structure_map_warmup_s_ = declare_parameter<double>(
      "structure_map_warmup_s", 5.0);
    structure_map_freeze_after_s_ = declare_parameter<double>(
      "structure_map_freeze_after_s", 15.0);
    structure_map_min_observations_ = declare_parameter<int>(
      "structure_map_min_observations", 3);
    structure_map_max_points_ = declare_parameter<int>(
      "structure_map_max_points", 200000);
    structure_z_min_ = declare_parameter<double>("structure_z_min", 0.55);
    structure_z_max_ = declare_parameter<double>("structure_z_max", 3.0);

    voxel_size_ = declare_parameter<double>("voxel_size", 0.1);  // <=0 disables

    enable_outlier_ = declare_parameter<bool>("enable_outlier", true);
    outlier_radius_ = declare_parameter<double>("outlier_radius", 0.5);
    outlier_min_neighbors_ =
      declare_parameter<int>("outlier_min_neighbors", 3);

    // Distance crop: only keep points within this radius (meters) from the
    // moving boat origin (0,0,0 in output frame). Drops far-away horizon /
    // shore points that make the costmap think the entire world is occupied.
    max_range_ = declare_parameter<double>("max_range", 30.0);
    raw_z_min_ = declare_parameter<double>("raw_z_min", 0.55);
    raw_z_max_ = declare_parameter<double>("raw_z_max", 4.0);
    // The old rectangular self-mask removed every return in the central
    // corridor (x[-2.8, 2.8], |y|<1.6).  That also removed a real obstacle
    // exactly when it reached the bow, so the emergency cloud went clear just
    // before impact.  Keep the legacy parameters for rollback, but use a
    // geometry-aware WAM-V mask by default: the two floats and the narrow
    // beams/deck are removed, while the open centreline remains observable.
    self_mask_mode_ = declare_parameter<std::string>(
      "self_mask_mode", "wamv_geometry");
    self_x_min_ = declare_parameter<double>("self_x_min", -2.8);
    self_x_max_ = declare_parameter<double>("self_x_max", 2.8);
    self_y_half_width_ = declare_parameter<double>("self_y_half_width", 1.6);

    log_throttle_ms_ = declare_parameter<int>("log_throttle_ms", 5000);
    structure_map_voxel_size_ = std::max(0.05, structure_map_voxel_size_);
    structure_map_warmup_s_ = std::max(0.0, structure_map_warmup_s_);
    structure_map_min_observations_ = std::max(1, structure_map_min_observations_);
    structure_map_max_points_ = std::max(1000, structure_map_max_points_);

    // Sensor-data QoS: best-effort, matches high-rate cloud publishers.
    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    pub_struct_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      structure_output_topic_, qos);
    if (!structure_map_output_topic_.empty()) {
      pub_struct_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        structure_map_output_topic_, qos);
    }
    pub_safety_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      safety_output_topic_, qos);
    pub_raw_obstacle_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_obstacle_output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&ObstacleFilter::cb, this, std::placeholders::_1));
    raw_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_input_topic_, qos,
      std::bind(&ObstacleFilter::raw_cb, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "obstacle_filter up:");
    RCLCPP_INFO(get_logger(), "  in : %s", input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "  raw: %s -> %s, %s", raw_input_topic_.c_str(),
      safety_output_topic_.c_str(), raw_obstacle_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  out: %s [%.2f, %.2f]", output_topic_.c_str(),
                z_min_, z_max_);
    RCLCPP_INFO(get_logger(), "  out: %s [%.2f, %.2f]",
                structure_output_topic_.c_str(), structure_z_min_,
                structure_z_max_);
    RCLCPP_INFO(get_logger(), "  map: %s frame=%s",
                structure_map_output_topic_.c_str(), world_output_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  out frame: %s, voxel %.2f m, max_range %.1f m",
                output_frame_.c_str(), voxel_size_, max_range_);
  }

private:
  // Return true only for points belonging to the WAM-V itself.  A single
  // rectangle is unsafe here: the vessel is a catamaran, so the water-level
  // space between the floats is a valid line of sight to an obstacle.  The
  // mask follows the collision geometry in wamv_base.urdf.xacro (float
  // centres at y=+/-1.03 m, narrow beams, and the small top deck).
  //
  // The raw LiDAR cloud is already transformed into base_link before this
  // function is called.  Obstacles in the centreline therefore survive even
  // when their nearest point is only 0.5--1.0 m in front of the bow.
  bool is_self_return(const PointT & pt) const
  {
    if (self_mask_mode_ != "wamv_geometry") {
      return pt.x >= self_x_min_ && pt.x <= self_x_max_ &&
             std::abs(pt.y) <= self_y_half_width_;
    }

    const double ay = std::abs(static_cast<double>(pt.y));
    const bool in_float =
      ay >= 0.72 && ay <= 1.34 &&
      pt.x >= -2.65f && pt.x <= 2.65f &&
      pt.z >= 0.0f && pt.z <= 0.85f;
    const bool in_beam =
      ay >= 0.55 && ay <= 1.16 &&
      pt.x >= -1.25f && pt.x <= 1.25f &&
      pt.z >= 0.45f && pt.z <= 1.35f;
    const bool in_top_deck =
      ay <= 0.58 &&
      pt.x >= -1.05f && pt.x <= 1.05f &&
      pt.z >= 1.10f && pt.z <= 1.40f;

    // VRX adds two CPU-case collision boxes on the centre deck.  They are
    // part of the WAM-V model (not an obstacle), and the 32-beam lidar sees
    // their upper faces as a dense near-field patch.  The first box is
    // centred at base_link x=0.035, z=1.53 with half-size (0.2975, 0.415,
    // 0.235); the second is centred at x=-0.45, z=1.438 with half-size
    // (0.1875, 0.32, 0.14).  Keep a small margin for Gazebo mesh/noise, but
    // do not expand this mask into the bow centreline: real obstacles in
    // front of the vessel must remain visible.
    const bool in_cpu_case_1 =
      pt.x >= -0.32f && pt.x <= 0.39f &&
      ay <= 0.45 &&
      pt.z >= 1.22f && pt.z <= 1.82f;
    const bool in_cpu_case_2 =
      pt.x >= -0.68f && pt.x <= -0.22f &&
      ay <= 0.36 &&
      pt.z >= 1.23f && pt.z <= 1.64f;

    return in_float || in_beam || in_top_deck ||
           in_cpu_case_1 || in_cpu_case_2;
  }

  // Run the shared filter pipeline on one band and publish to `out`.
  void publish_band(
    const sensor_msgs::msg::PointCloud2 & msg,
    double z_min, double z_max,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr out,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_out = nullptr)
  {
    auto cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(msg, *cloud);
    const std::size_t n0 = cloud->size();

    // 1. finite-point removal (Gazebo no-return rays are inf; is_dense may lie)
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

    // 2. Transform before both height and range cropping.  camera_init is an
    //    arbitrary LIO frame (its z origin is not guaranteed to be sea level),
    //    while base_link has a stable water-relative height.  Filtering z in
    //    camera_init caused the structure cloud to disappear as LIO z drifted.
    if (output_frame_ != msg.header.frame_id && !cloud->empty()) {
      try {
        // cloud_registered is an accumulated map and its header timestamp can
        // be older than the finite TF cache (this was observed as
        // 'timestamp ... earlier than all data in the transform cache').
        // Use the newest available pose, then stamp the transformed cloud now.
        // This keeps the local obstacle window alive while remaining
        // conservative: a failed lookup drops the frame rather than inventing
        // a transform.
        const auto tf = tf_buffer_->lookupTransform(
          output_frame_, msg.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(0.5));
        const Eigen::Affine3d T = tf2::transformToEigen(tf.transform);
        pcl::transformPointCloud(*cloud, *cloud, T);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "skip frame: %s", e.what());
        return;
      }
    }

    // 3. Height crop in the boat/output frame.  Water returns cluster around
    //    z=0; shore, docks, and buoys have returns above the configured band.
    {
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(static_cast<float>(z_min),
                           static_cast<float>(z_max));
      pass.filter(*cloud);
    }
    const std::size_t n_band = cloud->size();

    // 4. Remove the vessel itself from the registered Point-LIO cloud too.
    // Previously only the direct/raw scan had this mask, so startup hull/deck
    // returns could be confirmed into the persistent global structure map.
    if (!cloud->empty()) {
      auto no_self = std::make_shared<CloudT>();
      no_self->reserve(cloud->size());
      for (const auto & pt : cloud->points) {
        if (!is_self_return(pt)) {
          no_self->push_back(pt);
        }
      }
      cloud = no_self;
    }
    const std::size_t n_no_self = cloud->size();

    // 5. Distance crop in the boat/output frame.  This keeps the configured
    //    max_range local to the moving vessel, not to camera_init's origin.
    if (max_range_ > 0.0 && !cloud->empty()) {
      CloudT::Ptr cloud_near(new CloudT);
      cloud_near->reserve(cloud->size());
      const float r2_max = static_cast<float>(max_range_ * max_range_);
      for (const auto & pt : cloud->points) {
        const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 <= r2_max) {
          cloud_near->push_back(pt);
        }
      }
      cloud = cloud_near;
    }
    const std::size_t n_range = cloud->size();

    // 6. voxel-grid downsample
    if (voxel_size_ > 0.0 && !cloud->empty()) {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      const float leaf = static_cast<float>(voxel_size_);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter(*cloud);
    }

    // 7. radius outlier removal (spray / wave-crest specks)
    if (enable_outlier_ && cloud->size() > 10) {
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(cloud);
      ror.setRadiusSearch(outlier_radius_);
      ror.setMinNeighborsInRadius(outlier_min_neighbors_);
      ror.filter(*cloud);
    }
    const std::size_t n_out = cloud->size();

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    msg_out.header = msg.header;
    // The geometry was transformed with the newest available TF.  VRX can
    // advance simulated time faster than this PCL callback can process a
    // registered cloud, so a wall-clock/sensor stamp is already older than
    // Nav2's short TF cache by the time it arrives.  Zero requests the latest
    // transform in Nav2's MessageFilter and prevents valid transformed data
    // from being discarded as "earlier than all data in the transform cache".
    msg_out.header.stamp.sec = 0;
    msg_out.header.stamp.nanosec = 0;
    msg_out.header.frame_id = output_frame_;
    out->publish(msg_out);

    // Keep a second copy in the fixed world frame for the global costmap.
    // The local/base_link copy above must not be reused for this purpose: its
    // origin moves with the boat and would smear static shore points as the
    // vessel travels.  We undo the current base-link transform here after
    // filtering, so the height crop is still water-relative and the published
    // map points remain world-fixed.
    if (world_out) {
      auto world_cloud = std::make_shared<CloudT>(*cloud);
      if (world_output_frame_ != output_frame_ && !world_cloud->empty()) {
        try {
          const auto tf = tf_buffer_->lookupTransform(
            world_output_frame_, output_frame_, tf2::TimePointZero,
            tf2::durationFromSec(0.5));
          const Eigen::Affine3d T = tf2::transformToEigen(tf.transform);
          pcl::transformPointCloud(*world_cloud, *world_cloud, T);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "skip world structure copy: %s", e.what());
          return;
        }
      }
      // /cloud_registered is a single registered scan, not a persistent map.
      // Keep a world-fixed voxel map here so the dynamic tracker can subtract
      // shoreline/buoys that were observed on previous scans.  The map is
      // bootstrapped for a short, configurable interval, then frozen; this
      // prevents a later moving target from being learned as static geometry.
      if (structure_map_accumulate_) {
        const double now_s = get_clock()->now().seconds();
        if (!structure_map_started_) {
          structure_map_started_ = true;
          structure_map_start_s_ = now_s;
          RCLCPP_INFO(
            get_logger(),
            "structure-map warmup started: %.1f s, then %.1f s accumulation, min observations=%d",
            structure_map_warmup_s_, structure_map_freeze_after_s_,
            structure_map_min_observations_);
        }
        const double elapsed = std::max(0.0, now_s - structure_map_start_s_);
        const bool warming_up = elapsed < structure_map_warmup_s_;
        const double accumulation_elapsed =
          std::max(0.0, elapsed - structure_map_warmup_s_);
        const bool freeze_due =
          !warming_up && structure_map_freeze_after_s_ >= 0.0 &&
          accumulation_elapsed >= structure_map_freeze_after_s_;
        if (!structure_map_frozen_ && !warming_up && !freeze_due) {
          // Count each voxel at most once per cloud. A transient startup return
          // must be present in several independent frames before becoming a
          // permanent global obstacle.
          std::unordered_map<MapVoxelKey, PointT, MapVoxelKeyHash> frame_voxels;
          frame_voxels.reserve(world_cloud->size());
          for (const auto & point : world_cloud->points) {
            const MapVoxelKey key{
              static_cast<int>(std::floor(point.x / structure_map_voxel_size_)),
              static_cast<int>(std::floor(point.y / structure_map_voxel_size_)),
              static_cast<int>(std::floor(point.z / structure_map_voxel_size_))};
            frame_voxels.emplace(key, point);
          }
          for (const auto & entry : frame_voxels) {
            const int observations = ++structure_map_candidates_[entry.first];
            if (observations >= structure_map_min_observations_ &&
              structure_map_.find(entry.first) == structure_map_.end())
            {
              structure_map_.emplace(entry.first, entry.second);
              if (static_cast<int>(structure_map_.size()) >= structure_map_max_points_) {
                structure_map_frozen_ = true;
                break;
              }
            }
          }
        }
        if (!structure_map_frozen_ && freeze_due) {
          structure_map_frozen_ = true;
          structure_map_candidates_.clear();
          RCLCPP_INFO(
            get_logger(), "structure map frozen with %zu confirmed voxels",
            structure_map_.size());
        }
        world_cloud->clear();
        world_cloud->reserve(structure_map_.size());
        for (const auto & entry : structure_map_) {
          world_cloud->push_back(entry.second);
        }
      }

      sensor_msgs::msg::PointCloud2 world_msg;
      pcl::toROSMsg(*world_cloud, world_msg);
      world_msg.header.stamp.sec = 0;
      world_msg.header.stamp.nanosec = 0;
      world_msg.header.frame_id = world_output_frame_;
      world_out->publish(world_msg);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "filtered [%.2f,%.2f]: %zu -> band %zu -> no-self %zu -> range %zu -> out %zu",
      z_min, z_max, n0, n_band, n_no_self, n_range, n_out);
  }

  // Convert the current raw LiDAR scan into the boat frame. This output is
  // a conservative near-field cloud with only a water-height crop; no radius
  // outlier removal is applied because sparse shoreline returns still matter.
  void raw_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);
    if (cloud->empty()) {
      return;
    }

    // Keep the timestamp of the geometry consistent with the transform used
    // below.  The direct safety cloud is consumed by RViz as well as by the
    // control watchdog; stamping it with node->now() can put it ahead of the
    // latest camera_init->aft_mapped TF when Point-LIO is processing a large
    // registered cloud.  RViz then reports a SafetyCloud "Transform" error
    // even though the points themselves are valid.  A zero stamp means
    // "latest available transform" to TF2 message filters and is also safe
    // for the control watchdog, which measures receipt time independently.
    if (msg->header.frame_id != output_frame_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          output_frame_, msg->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(0.2));
        const auto eigen_tf = tf2::transformToEigen(tf.transform);
        const Eigen::Affine3f affine_tf = eigen_tf.cast<float>();
        pcl::transformPointCloud(*cloud, *cloud, affine_tf);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms_,
          "skip raw safety frame: %s", e.what());
        return;
      }
    }

    // Remove water returns and the vessel's own hull/deck, then retain a local
    // obstacle window. This direct scan is used by both Nav2 and the final
    // emergency gate, so it remains useful even when Point-LIO's accumulated
    // map is sparse or delayed.
    const float max_r2 = static_cast<float>(max_range_ * max_range_);
    auto near = std::make_shared<CloudT>();
    near->reserve(cloud->size());
    for (const auto & pt : cloud->points) {
      const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (!is_self_return(pt) && pt.z >= raw_z_min_ && pt.z <= raw_z_max_ &&
        (max_range_ <= 0.0 || r2 <= max_r2))
      {
        near->push_back(pt);
      }
    }
    cloud = near;

    sensor_msgs::msg::PointCloud2 safety_out;
    pcl::toROSMsg(*cloud, safety_out);
    safety_out.header.stamp.sec = 0;
    safety_out.header.stamp.nanosec = 0;
    safety_out.header.frame_id = output_frame_;
    pub_safety_->publish(safety_out);

    // Nav2 transforms this already body-frame cloud into camera_init.  As with
    // the registered structure cloud, request the newest transform so fast
    // VRX simulated time cannot age the observation out of the TF cache.  The
    // independent safety output above keeps a real timestamp; its watchdog
    // measures receipt age and does not depend on this costmap-only stamp.
    auto costmap_out = safety_out;
    costmap_out.header.stamp.sec = 0;
    costmap_out.header.stamp.nanosec = 0;
    pub_raw_obstacle_->publish(costmap_out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_throttle_ms_,
      "raw safety cloud: %zu finite -> %zu near points (frame %s)",
      idx.size(), cloud->size(), output_frame_.c_str());
  }

  void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    publish_band(*msg, z_min_, z_max_, pub_);
    publish_band(*msg, structure_z_min_, structure_z_max_, pub_struct_,
                 pub_struct_map_);
  }

  std::string input_topic_, raw_input_topic_, output_topic_,
    structure_output_topic_, structure_map_output_topic_, safety_output_topic_,
    raw_obstacle_output_topic_;
  std::string output_frame_, world_output_frame_;
  double z_min_, z_max_, structure_z_min_, structure_z_max_, voxel_size_;
  double structure_map_voxel_size_, structure_map_warmup_s_;
  double structure_map_freeze_after_s_;
  int structure_map_min_observations_;
  double max_range_;
  double raw_z_min_, raw_z_max_, self_x_min_, self_x_max_, self_y_half_width_;
  std::string self_mask_mode_;
  bool enable_outlier_, structure_map_accumulate_;
  double outlier_radius_;
  int structure_map_max_points_;
  bool structure_map_started_{false};
  bool structure_map_frozen_{false};
  double structure_map_start_s_{0.0};
  std::unordered_map<MapVoxelKey, PointT, MapVoxelKeyHash> structure_map_;
  std::unordered_map<MapVoxelKey, int, MapVoxelKeyHash> structure_map_candidates_;
  int outlier_min_neighbors_;
  int log_throttle_ms_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_struct_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_struct_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_safety_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw_obstacle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
