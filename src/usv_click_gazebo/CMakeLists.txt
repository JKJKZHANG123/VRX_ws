cmake_minimum_required(VERSION 3.16)
project(usv_click_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# AUTOMOC needed for the plugin's Q_OBJECT (Q_INVOKABLE methods for QML buttons)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

# ── gz GUI plugin: captures LeftClickToScene -> /gazebo/click/goal ──
# gz-gui requires a QML file (compiled into the .so via .qrc) for the plugin
# to load cleanly; without it LoadConfig never runs.
qt5_add_resources(GZ_CLICK_QRC_SOURCES src/gz_click_to_goal.qrc)

add_library(gz_click_to_goal SHARED
  src/gz_click_to_goal.cc
  ${GZ_CLICK_QRC_SOURCES}
)
target_link_libraries(gz_click_to_goal
  gz-gui8::gz-gui8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
  Qt5::Core
  Qt5::Quick
)

install(TARGETS gz_click_to_goal
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# ── Python node: world -> odom goal conversion ───────────────────
install(PROGRAMS
  usv_click_gazebo/gz_click_to_goal_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
