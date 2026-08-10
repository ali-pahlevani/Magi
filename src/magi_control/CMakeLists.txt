cmake_minimum_required(VERSION 3.8)
project(magi_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/magi_posture.py
    scripts/magi_balance.py
    scripts/magi_drive_benchmark.py
    scripts/magi_stability_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
