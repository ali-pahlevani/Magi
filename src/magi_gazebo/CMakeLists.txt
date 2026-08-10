cmake_minimum_required(VERSION 3.8)
project(magi_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config models scripts
  DESTINATION share/${PROJECT_NAME}
)

# Put the vendored Rubicon model on GZ_SIM_RESOURCE_PATH for anyone sourcing
# this workspace, so `model://Rubicon` resolves without a Fuel download.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
