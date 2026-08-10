cmake_minimum_required(VERSION 3.8)
project(magi_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/magi_unpause.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
