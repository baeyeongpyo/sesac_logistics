cmake_minimum_required(VERSION 3.8)
project(mentorpi_fleet)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY mentorpi_fleet DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  mentorpi_fleet/fleet_manager.py
  mentorpi_fleet/simulation_manager.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_registry test/test_registry.py)
  ament_add_pytest_test(test_bridge_config test/test_bridge_config.py)
  ament_add_pytest_test(test_worker_manager test/test_worker_manager.py)
  ament_add_pytest_test(test_fleet_state test/test_fleet_state.py)
  ament_add_pytest_test(test_simulation_manager test/test_simulation_manager.py)
endif()

ament_package()
