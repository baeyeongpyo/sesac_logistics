cmake_minimum_required(VERSION 3.8)
project(mentorpi_map_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/map_visualization_tf.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
