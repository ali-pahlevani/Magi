cmake_minimum_required(VERSION 3.8)
project(magi_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/magi_imu_covariance.py
    scripts/magi_yaw_compare.py
    scripts/magi_leg_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
