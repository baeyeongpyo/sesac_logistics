cmake_minimum_required(VERSION 3.8)
project(orthogonal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(orthogonal_controller SHARED
  src/orthogonal_controller.cpp
)

target_include_directories(orthogonal_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orthogonal_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(
  nav2_core
  orthogonal_controller_plugin.xml
)

install(
  TARGETS orthogonal_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES orthogonal_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()