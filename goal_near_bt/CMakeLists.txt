cmake_minimum_required(VERSION 3.8)
project(goal_near_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_library(goal_near_bt_plugin SHARED
  src/goal_near_condition.cpp
)

target_compile_definitions(goal_near_bt_plugin PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(goal_near_bt_plugin
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)

install(
  TARGETS goal_near_bt_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()