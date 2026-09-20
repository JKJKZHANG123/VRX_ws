#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>
#include <cmath>
#include <array>
#include <memory>

class GpsTrajectoryNode : public rclcpp::Node
{
public:
    GpsTrajectoryNode() : Node("gps_trajectory_node")
    {
        // 声明参数
        this->declare_parameter("window_size", 5);
        this->declare_parameter("max_path_length", 1000);
        this->declare_parameter("scale_factor", 100000.0);
        this->declare_parameter("min_distance_threshold", 1.0);  // 新增：最小距离阈值（单位：米）
        this->declare_parameter("min_accuracy_threshold", 5.0);  // 新增：精度阈值（单位：米）
        
        // 获取参数
        window_size_ = this->get_parameter("window_size").as_int();
        max_path_length_ = this->get_parameter("max_path_length").as_int();
        scale_factor_ = this->get_parameter("scale_factor").as_double();
        min_distance_threshold_ = this->get_parameter("min_distance_threshold").as_double();
        min_accuracy_threshold_ = this->get_parameter("min_accuracy_threshold").as_double();
        
        // 订阅GPS数据
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&GpsTrajectoryNode::gpsCallback, this, std::placeholders::_1));
        
        // 发布轨迹路径
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);
        
        // 发布轨迹标记（可选）
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/gps/trajectory_marker", 10);
        
        // 初始化路径消息
        path_.header.frame_id = "map";
        
        RCLCPP_INFO(this->get_logger(), "GPS轨迹节点启动");
        RCLCPP_INFO(this->get_logger(), "参数: window_size=%d, max_path_length=%d, scale_factor=%.1f",
                   window_size_, max_path_length_, scale_factor_);
        RCLCPP_INFO(this->get_logger(), "阈值: 距离=%.2fm, 精度=%.2fm",
                   min_distance_threshold_, min_accuracy_threshold_);
    }

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // 检查GPS数据是否有效
        if (msg->status.status < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "GPS数据无效，状态: %d", msg->status.status);
            return;
        }
        
        // 检查经纬度是否为有效值
        if (std::abs(msg->latitude) < 1e-6 && std::abs(msg->longitude) < 1e-6) {
            return;
        }
        
        // 检查GPS精度（位置协方差）
        double accuracy_horizontal = std::sqrt(msg->position_covariance[0]);
        double accuracy_vertical = std::sqrt(msg->position_covariance[8]);
        
        if (accuracy_horizontal > min_accuracy_threshold_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "GPS精度不足: 水平=%.2fm, 垂直=%.2fm",
                                accuracy_horizontal, accuracy_vertical);
            return;
        }
        
        try {
            // 如果是第一个有效点，设置为原点
            if (!origin_set_) {
                origin_lat_ = msg->latitude;
                origin_lon_ = msg->longitude;
                origin_alt_ = std::isnan(msg->altitude) ? 0.0 : msg->altitude;
                origin_set_ = true;
                
                RCLCPP_INFO(this->get_logger(), "设置原点: 纬度=%.6f°, 经度=%.6f°, 海拔=%.2fm",
                           origin_lat_, origin_lon_, origin_alt_);
            }
            
            // 简单的相对坐标计算
            // 使用经纬度的差值作为相对坐标（乘以一个缩放因子）
            double x = (msg->longitude - origin_lon_) * scale_factor_;
            double y = (msg->latitude - origin_lat_) * scale_factor_;
            double z = std::isnan(msg->altitude) ? 0.0 : msg->altitude - origin_alt_;
            
            // 使用滑动窗口平滑轨迹
            smoothing_window_.push_back({x, y, z});
            if (smoothing_window_.size() > window_size_) {
                smoothing_window_.pop_front();
            }
            
            // 计算平滑后的坐标
            std::array<double, 3> smoothed = {0.0, 0.0, 0.0};
            if (!smoothing_window_.empty()) {
                for (const auto& point : smoothing_window_) {
                    smoothed[0] += point[0];
                    smoothed[1] += point[1];
                    smoothed[2] += point[2];
                }
                smoothed[0] /= smoothing_window_.size();
                smoothed[1] /= smoothing_window_.size();
                smoothed[2] /= smoothing_window_.size();
            } else {
                smoothed = {x, y, z};
            }
            
            // 计算与上一个点的距离（如果存在）
            if (!path_.poses.empty()) {
                const auto& last_pose = path_.poses.back();
                double dx = smoothed[0] - last_pose.pose.position.x;
                double dy = smoothed[1] - last_pose.pose.position.y;
                double dz = smoothed[2] - last_pose.pose.position.z;
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                // 如果移动距离太小，忽略这个点（减少漂移影响）
                if (distance < min_distance_threshold_) {
                    // 可选：更新最后一个点的时间戳，但不添加新点
                    path_.header.stamp = this->now();
                    return;
                }
            }
            
            // 创建轨迹点
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = smoothed[0];
            pose.pose.position.y = smoothed[1];
            pose.pose.position.z = smoothed[2];
            pose.pose.orientation.w = 1.0;  // 无旋转
            
            // 添加到路径
            path_.poses.push_back(pose);
            path_.header.stamp = pose.header.stamp;
            
            // 限制路径长度
            if (path_.poses.size() > max_path_length_) {
                path_.poses.erase(path_.poses.begin());
            }
            
            // 发布路径
            path_pub_->publish(path_);
            
            // 可选：发布轨迹标记
            publishTrajectoryMarker();
            
            // 可选：记录最后几个点的信息
            if (path_.poses.size() % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "轨迹点数量: %zu, 当前精度: 水平=%.2fm",
                           path_.poses.size(), accuracy_horizontal);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "坐标处理错误: %s", e.what());
        }
    }
    
    void publishTrajectoryMarker()
    {
        if (path_.poses.empty()) return;
        
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();
        
        marker->header.frame_id = "map";
        marker->header.stamp = this->now();
        marker->ns = "gps_trajectory";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker->action = visualization_msgs::msg::Marker::ADD;
        
        // 设置线宽
        marker->scale.x = 0.1;
        
        // 设置颜色（蓝色）
        marker->color.a = 1.0;
        marker->color.r = 0.0;
        marker->color.g = 0.0;
        marker->color.b = 1.0;
        
        // 添加轨迹点
        for (const auto& pose : path_.poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            marker->points.push_back(point);
        }
        
        marker_pub_->publish(std::move(marker));
    }
    
    // ROS2订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    // 轨迹数据
    nav_msgs::msg::Path path_;
    
    // 原点坐标
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;
    bool origin_set_ = false;
    
    // 轨迹参数
    std::deque<std::array<double, 3>> smoothing_window_;
    int window_size_;
    int max_path_length_;
    double scale_factor_;
    double min_distance_threshold_;  // 最小距离阈值
    double min_accuracy_threshold_;  // 最小精度阈值
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsTrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}