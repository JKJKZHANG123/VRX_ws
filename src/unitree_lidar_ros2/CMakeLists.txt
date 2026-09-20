cmake_minimum_required(VERSION 3.10)
project(unitree_lidar_ros2)

# Jazzy 推荐 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_include_directories(unitree_lidar_ros2_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_lidar_sdk/include
)

# 更规范：用 IMPORTED 明确静态库路径（不再写 libxxx.a 作为“裸名字”）
add_library(unitree_lidar_sdk STATIC IMPORTED)
set_target_properties(unitree_lidar_sdk PROPERTIES
  IMPORTED_LOCATION
  "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_lidar_sdk/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a"
)

target_link_libraries(unitree_lidar_ros2_node
  unitree_lidar_sdk
  ${PCL_LIBRARIES}
)

ament_target_dependencies(unitree_lidar_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

install(TARGETS unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# 更规范：目录安装（避免路径写错）
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
