cmake_minimum_required(VERSION 3.8)
project(mentorpi_gz_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gz_pose_to_odom.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_harmonic_launch_contract
    test/test_harmonic_launch_contract.py)
endif()

ament_package()
