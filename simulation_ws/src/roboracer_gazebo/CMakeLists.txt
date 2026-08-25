cmake_minimum_required(VERSION 3.0.2)
project(roboracer_gazebo)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/gazebo_vehicle_bridge.py
  scripts/gazebo_opponent_driver.py
  scripts/mid360_imu_adapter.py
  scripts/sim_enable_controller.py
  scripts/simulation_healthcheck.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch urdf worlds config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
