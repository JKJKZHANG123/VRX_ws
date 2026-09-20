cmake_minimum_required(VERSION 3.8)
project(my_gps_driver)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加新的依赖（轨迹绘制所需）
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 创建可执行文件：GPS轨迹节点
add_executable(gps_trajectory_node src/gps_trajectory_node.cpp)

# 为可执行文件添加依赖
ament_target_dependencies(gps_trajectory_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 如果有其他节点，也可以添加（例如gps_driver_node）
# add_executable(gps_driver_node src/gps_driver_node.cpp)
# ament_target_dependencies(gps_driver_node
#   rclcpp
#   nmea_msgs
#   sensor_msgs
# )

# 安装可执行文件
install(TARGETS
  gps_trajectory_node
  # gps_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(
  rclcpp
  nmea_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()