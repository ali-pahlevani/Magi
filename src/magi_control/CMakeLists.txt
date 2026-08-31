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
    scripts/magi_stabilizer.py
    scripts/magi_drive_benchmark.py
    scripts/magi_stability_test.py
    scripts/magi_terrain_trial.py
  DESTINATION lib/${PROJECT_NAME}
)

# Shared leg model, imported by magi_balance.py and magi_stabilizer.py. It goes
# in the same directory as the scripts so a plain `import magi_leg_model`
# resolves -- Python puts a script's own directory on sys.path, and both the
# install tree and the source tree keep them together.
install(
  FILES scripts/magi_leg_model.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
