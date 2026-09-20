/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "unitree_lidar_sdk_pcl.h"

// double 秒 -> builtin_interfaces::msg::Time（跨版本最稳）
inline builtin_interfaces::msg::Time to_builtin_time(double stamp_sec)
{
  builtin_interfaces::msg::Time t;
  const int64_t sec = static_cast<int64_t>(stamp_sec);
  const double frac = stamp_sec - static_cast<double>(sec);
  int64_t nsec = static_cast<int64_t>(frac * 1000000000.0);

  // 防止浮点误差导致越界
  if (nsec < 0) nsec = 0;
  if (nsec >= 1000000000LL) {
    t.sec = static_cast<int32_t>(sec + 1);
    t.nanosec = static_cast<uint32_t>(nsec - 1000000000LL);
  } else {
    t.sec = static_cast<int32_t>(sec);
    t.nanosec = static_cast<uint32_t>(nsec);
  }
  return t;
}

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
  explicit UnitreeLidarSDKNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UnitreeLidarSDKNode() override = default;

  void timer_callback();

protected:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Unitree Lidar Reader
  UnitreeLidarReader * lsdk_{nullptr};

  // Config params
  std::string port_{"/dev/ttyUSB0"};

  double rotate_yaw_bias_{0.0};
  double range_scale_{0.001};
  double range_bias_{0.0};
  double range_max_{50.0};
  double range_min_{0.0};

  std::string cloud_frame_{"unilidar_lidar"};
  std::string cloud_topic_{"unilidar/cloud"};
  int cloud_scan_num_{18};

  std::string imu_frame_{"unilidar_imu"};
  std::string imu_topic_{"unilidar/imu"};
};

///////////////////////////////////////////////////////////////////

inline UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions & options)
: Node("unitree_lidar_sdk_node", options)
{
  // parameters
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");

  this->declare_parameter<double>("rotate_yaw_bias", 0.0);
  this->declare_parameter<double>("range_scale", 0.001);
  this->declare_parameter<double>("range_bias", 0.0);
  this->declare_parameter<double>("range_max", 50.0);
  this->declare_parameter<double>("range_min", 0.0);

  this->declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
  this->declare_parameter<std::string>("cloud_topic", "unilidar/cloud");
  this->declare_parameter<int>("cloud_scan_num", 18);

  this->declare_parameter<std::string>("imu_frame", "unilidar_imu");
  this->declare_parameter<std::string>("imu_topic", "unilidar/imu");

  port_ = this->get_parameter("port").as_string();

  rotate_yaw_bias_ = this->get_parameter("rotate_yaw_bias").as_double();
  range_scale_ = this->get_parameter("range_scale").as_double();
  range_bias_ = this->get_parameter("range_bias").as_double();
  range_max_ = this->get_parameter("range_max").as_double();
  range_min_ = this->get_parameter("range_min").as_double();

  cloud_frame_ = this->get_parameter("cloud_frame").as_string();
  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  cloud_scan_num_ = static_cast<int>(this->get_parameter("cloud_scan_num").as_int());

  imu_frame_ = this->get_parameter("imu_frame").as_string();
  imu_topic_ = this->get_parameter("imu_topic").as_string();

  // Init Unitree SDK
  lsdk_ = createUnitreeLidarReader();
  if (!lsdk_) {
    throw std::runtime_error("Failed to create UnitreeLidarReader");
  }
  lsdk_->initialize(
    cloud_scan_num_, port_, 2000000, rotate_yaw_bias_,
    range_scale_, range_bias_, range_max_, range_min_);

// QoS: make both reliable to avoid QoS mismatch with RViz/Point-LIO
  auto qos_cloud = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto qos_imu   = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();

  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, qos_cloud);
  pub_imu_   = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos_imu);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&UnitreeLidarSDKNode::timer_callback, this));
}

inline void UnitreeLidarSDKNode::timer_callback()
{
  const MessageType result = lsdk_->runParse();
  static pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  if (result == IMU) {
    auto & imu = lsdk_->getIMU();

    sensor_msgs::msg::Imu imuMsg;
    imuMsg.header.frame_id = imu_frame_;
    imuMsg.header.stamp = to_builtin_time(imu.stamp);

    imuMsg.orientation.x = imu.quaternion[0];
    imuMsg.orientation.y = imu.quaternion[1];
    imuMsg.orientation.z = imu.quaternion[2];
    imuMsg.orientation.w = imu.quaternion[3];

    imuMsg.angular_velocity.x = imu.angular_velocity[0];
    imuMsg.angular_velocity.y = imu.angular_velocity[1];
    imuMsg.angular_velocity.z = imu.angular_velocity[2];

    imuMsg.linear_acceleration.x = imu.linear_acceleration[0];
    imuMsg.linear_acceleration.y = imu.linear_acceleration[1];
    imuMsg.linear_acceleration.z = imu.linear_acceleration[2];

    pub_imu_->publish(imuMsg);
  } else if (result == POINTCLOUD) {
    auto & cloud = lsdk_->getCloud();
    transformUnitreeCloudToPCL(cloud, cloudOut);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloudOut, cloud_msg);
    cloud_msg.header.frame_id = cloud_frame_;
    cloud_msg.header.stamp = to_builtin_time(cloud.stamp);

    pub_cloud_->publish(cloud_msg);
  }
}
