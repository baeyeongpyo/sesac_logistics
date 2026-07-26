cmake_minimum_required(VERSION 3.8)
project(mentorpi_slam)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/atomic_publish.py
  scripts/session_artifacts.py
  scripts/run_mapping_session.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_slam_contract
    test/test_slam_contract.py)
  ament_add_pytest_test(test_session_artifacts
    test/test_session_artifacts.py)
  ament_add_pytest_test(test_atomic_publish
    test/test_atomic_publish.py)
  ament_add_pytest_test(test_mapping_session_script
    test/test_mapping_session_script.py
    TIMEOUT 180)
endif()

ament_package()
