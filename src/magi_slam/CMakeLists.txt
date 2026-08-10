cmake_minimum_required(VERSION 3.8)
project(magi_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/magi_slam_accuracy.py
    scripts/magi_map_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
