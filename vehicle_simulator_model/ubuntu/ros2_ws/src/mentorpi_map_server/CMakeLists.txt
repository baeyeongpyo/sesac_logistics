cmake_minimum_required(VERSION 3.8)
project(mentorpi_map_server)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/map_visualization_tf.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_server_launch_contract
    test/test_map_server_launch_contract.py)
endif()

ament_package()
