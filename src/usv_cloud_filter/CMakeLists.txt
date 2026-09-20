cmake_minimum_required(VERSION 3.8)
project(usv_cloud_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(obstacle_filter src/obstacle_filter.cpp)
ament_target_dependencies(obstacle_filter
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_eigen
)
target_link_libraries(obstacle_filter ${PCL_LIBRARIES})
target_include_directories(obstacle_filter PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS obstacle_filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
