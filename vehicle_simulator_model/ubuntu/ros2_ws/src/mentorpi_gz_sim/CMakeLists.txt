cmake_minimum_required(VERSION 3.8)
project(mentorpi_gz_sim)
find_package(ament_cmake REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(pallet_manager_core STATIC src/pallet_command.cpp)
target_compile_features(pallet_manager_core PUBLIC cxx_std_17)
target_include_directories(pallet_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pallet_manager_core PUBLIC
  gz-common5::gz-common5
  gz-sim8::gz-sim8)

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gz_pose_to_odom.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS pallet_manager_core ARCHIVE DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_pallet_command test/test_pallet_command.cpp)
  target_link_libraries(test_pallet_command pallet_manager_core)
  ament_add_pytest_test(test_harmonic_launch_contract
    test/test_harmonic_launch_contract.py)
  ament_add_pytest_test(test_gz_pose_to_odom
    test/test_gz_pose_to_odom.py)
endif()

ament_package()
