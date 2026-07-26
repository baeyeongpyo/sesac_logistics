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
ament_package()
