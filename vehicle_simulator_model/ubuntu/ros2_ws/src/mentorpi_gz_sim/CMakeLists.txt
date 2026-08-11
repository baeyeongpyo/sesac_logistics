cmake_minimum_required(VERSION 3.8)
project(mentorpi_gz_sim)
find_package(ament_cmake REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(pallet_manager_core STATIC
  src/pallet_command.cpp
  src/pallet_registry.cpp
  src/pallet_model_factory.cpp)
set_target_properties(pallet_manager_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(pallet_manager_core PUBLIC cxx_std_17)
target_include_directories(pallet_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pallet_manager_core PUBLIC
  gz-common5::gz-common5
  gz-sim8::gz-sim8
  sdformat14::sdformat14)

add_library(mentorpi_warehouse_pallet_manager SHARED
  src/warehouse_pallet_manager.cpp)
target_compile_features(mentorpi_warehouse_pallet_manager PUBLIC cxx_std_17)
target_link_libraries(mentorpi_warehouse_pallet_manager PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  sdformat14::sdformat14
  pallet_manager_core)

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gz_pose_to_odom.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  pallet_manager_core
  mentorpi_warehouse_pallet_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/mentorpi_gz_sim.sh.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_pallet_command test/test_pallet_command.cpp)
  target_link_libraries(test_pallet_command pallet_manager_core)
  ament_add_gtest(test_pallet_registry test/test_pallet_registry.cpp)
  target_link_libraries(test_pallet_registry pallet_manager_core)
  target_compile_definitions(test_pallet_registry PRIVATE
    PALLET_TEMPLATE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models/pallet")
  ament_add_pytest_test(test_harmonic_launch_contract
    test/test_harmonic_launch_contract.py)
  ament_add_pytest_test(test_gz_pose_to_odom
    test/test_gz_pose_to_odom.py)
  ament_add_pytest_test(test_vehicle_adapter_contract
    test/test_vehicle_adapter_contract.py)
  ament_add_pytest_test(
    test_pallet_manager_integration
    test/test_pallet_manager_integration.py
    TIMEOUT 60
    APPEND_ENV
      "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:mentorpi_warehouse_pallet_manager>"
      "GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()

ament_package()
