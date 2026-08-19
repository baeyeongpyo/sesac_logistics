cmake_minimum_required(VERSION 3.8)
project(mentorpi_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/cmd_vel_relay.py
  scripts/cmd_vel_mux.py
  scripts/velocity_arbitration.py
  scripts/goal_bridge.py
  scripts/map_import.py
  scripts/map_session.py
  scripts/run_navigation.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_map_session test/test_map_session.py)
  ament_add_pytest_test(test_map_import test/test_map_import.py)
  ament_add_pytest_test(test_navigation_contract test/test_navigation_contract.py)
  ament_add_pytest_test(test_velocity_arbitration test/test_velocity_arbitration.py)
  ament_add_pytest_test(test_vehicle_navigation_contract test/test_vehicle_navigation_contract.py)
endif()

ament_package()
