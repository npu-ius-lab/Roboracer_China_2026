cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_dynamic_mpcc)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  rospy
  std_msgs
  std_srvs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

# The car uses the historical standalone residual workspace. The local archive
# also carries an equivalent source snapshot; use it only when the standalone
# development library is absent so candidate builds remain reproducible.
set(RESIDUAL_STANDALONE_ROOT "/home/tianbot/f1tenth_residual_ws")
set(RESIDUAL_SNAPSHOT_ROOT "/home/tianbot/f1tenth/code/f1tenth_residual_dynamics_patch")
if(EXISTS "${RESIDUAL_STANDALONE_ROOT}/devel/lib/libf1tenth_residual_model.so")
  set(RESIDUAL_MODEL_INCLUDE "${RESIDUAL_STANDALONE_ROOT}/src/f1tenth_residual_dynamics/include")
  set(RESIDUAL_MODEL_LIBRARY "${RESIDUAL_STANDALONE_ROOT}/devel/lib/libf1tenth_residual_model.so")
elseif(EXISTS "${RESIDUAL_SNAPSHOT_ROOT}/src/residual_model.cpp")
  add_library(f1tenth_residual_model_snapshot
    "${RESIDUAL_SNAPSHOT_ROOT}/src/residual_model.cpp")
  target_compile_features(f1tenth_residual_model_snapshot PUBLIC cxx_std_17)
  target_include_directories(f1tenth_residual_model_snapshot PUBLIC
    "${RESIDUAL_SNAPSHOT_ROOT}/include")
  target_link_libraries(f1tenth_residual_model_snapshot PUBLIC yaml-cpp)
  set(RESIDUAL_MODEL_INCLUDE "${RESIDUAL_SNAPSHOT_ROOT}/include")
  set(RESIDUAL_MODEL_LIBRARY f1tenth_residual_model_snapshot)
else()
  message(FATAL_ERROR "No residual-model C++ library or source snapshot found")
endif()

# vendored jsk_recognition_msgs (PolygonArray) used by PatchWork++
add_subdirectory(include/jsk_recognition_msgs)

catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES f1tenth_mpcc_core f1tenth_mpcc_acados_runtime
  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp roslib sensor_msgs std_msgs std_srvs visualization_msgs
  DEPENDS EIGEN3
)

include_directories(include
  ${CATKIN_DEVEL_PREFIX}/include
  /home/tianbot/tianbot_ws/devel/include
  ${RESIDUAL_MODEL_INCLUDE}
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(f1tenth_mpcc_core src/core.cpp src/pure_pursuit_candidate.cpp)
target_compile_features(f1tenth_mpcc_core PUBLIC cxx_std_17)
target_link_libraries(f1tenth_mpcc_core PUBLIC yaml-cpp
  ${RESIDUAL_MODEL_LIBRARY})

add_library(f1tenth_mpcc_acados_runtime src/acados_runtime.cpp)
target_compile_features(f1tenth_mpcc_acados_runtime PUBLIC cxx_std_17)
target_link_libraries(f1tenth_mpcc_acados_runtime PUBLIC f1tenth_mpcc_core ${CMAKE_DL_LIBS})

# Isolated local-reference runtime used by V5 Chaoche and the additive V3
# Pro Max React controller. Keep it linked against the current core library so
# CommandManager ABI changes rebuild the executable and runtime together.
add_library(f1tenth_mpcc_acados_runtime_chaoche
  src/acados_runtime_chaoche.cpp)
target_compile_features(f1tenth_mpcc_acados_runtime_chaoche PUBLIC cxx_std_17)
target_link_libraries(f1tenth_mpcc_acados_runtime_chaoche PUBLIC
  f1tenth_mpcc_core ${CMAKE_DL_LIBS})

# Restored isolated local-reference runtime used only by the RoboRacer H2H
# candidates. Stable V3/RoboRacer continue to link the normal runtime above.
add_library(f1tenth_mpcc_acados_runtime_jubu src/acados_runtime_jubu.cpp)
target_compile_features(f1tenth_mpcc_acados_runtime_jubu PUBLIC cxx_std_17)
target_link_libraries(f1tenth_mpcc_acados_runtime_jubu PUBLIC
  f1tenth_mpcc_core ${CMAKE_DL_LIBS})

add_library(f1tenth_v5_jubu_roi src/v5_jubu_roi.cpp)
target_compile_features(f1tenth_v5_jubu_roi PUBLIC cxx_std_17)

add_executable(mpcc_node_cpp src/mpcc_node.cpp)
target_compile_features(mpcc_node_cpp PUBLIC cxx_std_17)
target_link_libraries(mpcc_node_cpp f1tenth_mpcc_acados_runtime ${catkin_LIBRARIES})

# Isolated executable for the automatic-relaunch candidate. The frozen
# mpcc_node.cpp used by stableV3 and the accepted 5/3 m/s launcher is untouched.
add_executable(mpcc_node_auto_relaunch_cpp src/mpcc_node_auto_relaunch.cpp)
target_compile_features(mpcc_node_auto_relaunch_cpp PUBLIC cxx_std_17)
target_link_libraries(mpcc_node_auto_relaunch_cpp f1tenth_mpcc_acados_runtime ${catkin_LIBRARIES})

add_executable(mpcc_node_auto_relaunch_chaoche_cpp
  src/mpcc_node_auto_relaunch_chaoche.cpp)
target_compile_features(mpcc_node_auto_relaunch_chaoche_cpp PUBLIC cxx_std_17)
target_compile_definitions(mpcc_node_auto_relaunch_chaoche_cpp PRIVATE
  F1TENTH_CHAOCHE_CORE_ABI_REV=2)
target_link_libraries(mpcc_node_auto_relaunch_chaoche_cpp
  f1tenth_mpcc_acados_runtime_chaoche ${catkin_LIBRARIES})

add_executable(mpcc_node_auto_relaunch_jubu_cpp
  src/mpcc_node_auto_relaunch_jubu.cpp)
target_compile_features(mpcc_node_auto_relaunch_jubu_cpp PUBLIC cxx_std_17)
target_link_libraries(mpcc_node_auto_relaunch_jubu_cpp
  f1tenth_mpcc_acados_runtime_jubu ${catkin_LIBRARIES})

add_executable(v5_jubu_local_planner_cpp src/v5_jubu_local_planner.cpp)
target_compile_features(v5_jubu_local_planner_cpp PUBLIC cxx_std_17)
target_link_libraries(v5_jubu_local_planner_cpp
  f1tenth_v5_jubu_roi f1tenth_mpcc_core ${catkin_LIBRARIES})

add_executable(roboracer_h2h_follow_gate_cpp
  src/roboracer_h2h_follow_gate.cpp)
target_compile_features(roboracer_h2h_follow_gate_cpp PUBLIC cxx_std_17)
target_link_libraries(roboracer_h2h_follow_gate_cpp ${catkin_LIBRARIES})

# Isolated RoboRacer high-speed candidate with dual-rate command deceleration.
# The frozen Stable RoboRacer executable and launch remain unchanged.
add_executable(mpcc_node_auto_relaunch_dual_decel_cpp
  src/mpcc_node_auto_relaunch_dual_decel.cpp)
target_compile_features(mpcc_node_auto_relaunch_dual_decel_cpp PUBLIC cxx_std_17)
target_link_libraries(mpcc_node_auto_relaunch_dual_decel_cpp
  f1tenth_mpcc_acados_runtime ${catkin_LIBRARIES})

# StableV5 owns a separate binary so its tracking-quality speed gate can
# evolve without changing the frozen StableV3/V4 automatic-relaunch source.
add_executable(mpcc_node_stable_v5_cpp src/mpcc_node_stable_v5.cpp)
target_compile_features(mpcc_node_stable_v5_cpp PUBLIC cxx_std_17)
target_link_libraries(mpcc_node_stable_v5_cpp f1tenth_mpcc_acados_runtime ${catkin_LIBRARIES})

add_executable(ackermann_command_gate_cpp src/ackermann_command_gate.cpp)
target_compile_features(ackermann_command_gate_cpp PUBLIC cxx_std_17)
target_link_libraries(ackermann_command_gate_cpp ${catkin_LIBRARIES})

add_executable(lap_timer_node_cpp src/lap_timer_node.cpp)
target_compile_features(lap_timer_node_cpp PUBLIC cxx_std_17)
target_link_libraries(lap_timer_node_cpp f1tenth_mpcc_core ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  scripts/mpcc_node.py
  scripts/ackermann_command_gate.py
  scripts/inspect_raceline.py
  scripts/replay_odom_bag.py
  scripts/identify_actuators.py
  scripts/fit_tire_stiffness.py
  scripts/run_closed_loop_sim.py
  scripts/rviz_closed_loop_sim.py
  scripts/validate_vehicle_model.py
  scripts/replay_delay_compensation.py
  scripts/generate_acados_solvers.py
  scripts/validate_track_geometry.py
  scripts/telemetry_logger.py
  scripts/checkpoint_start_guard.py
  scripts/automatic_relaunch_supervisor.py
  scripts/automatic_relaunch_supervisor_race_start_v2.py
  scripts/automatic_relaunch_supervisor_stable_roboracer.py
  scripts/automatic_relaunch_supervisor_stable_roboracer_safe_start_candidate.py
  scripts/automatic_relaunch_pure_pursuit_debug_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config data docs launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS f1tenth_mpcc_core f1tenth_mpcc_acados_runtime
  f1tenth_mpcc_acados_runtime_chaoche
  f1tenth_mpcc_acados_runtime_jubu f1tenth_v5_jubu_roi
  mpcc_node_cpp mpcc_node_auto_relaunch_cpp
  mpcc_node_auto_relaunch_chaoche_cpp mpcc_node_auto_relaunch_jubu_cpp
  v5_jubu_local_planner_cpp roboracer_h2h_follow_gate_cpp
  mpcc_node_auto_relaunch_dual_decel_cpp mpcc_node_stable_v5_cpp
  ackermann_command_gate_cpp lap_timer_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(cpp_v5_chaoche_local_reference
    tests/cpp_v5_chaoche_local_reference.cpp)
  target_compile_features(cpp_v5_chaoche_local_reference PUBLIC cxx_std_17)
  target_link_libraries(cpp_v5_chaoche_local_reference
    f1tenth_mpcc_acados_runtime_chaoche)
  add_test(NAME cpp_v5_chaoche_local_reference COMMAND
    $<TARGET_FILE:cpp_v5_chaoche_local_reference>
    ${CMAKE_CURRENT_SOURCE_DIR}/config/candidates/stable_v5/controller.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/candidates/stable_v5/vehicle.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tracks/racelinev3_stable_v5_fast5_std32/raceline.csv)

  add_executable(cpp_core_smoke tests/cpp_core_smoke.cpp)
  target_compile_features(cpp_core_smoke PUBLIC cxx_std_17)
  target_link_libraries(cpp_core_smoke f1tenth_mpcc_core)
  add_test(NAME cpp_core_smoke COMMAND $<TARGET_FILE:cpp_core_smoke>
    ${CMAKE_CURRENT_SOURCE_DIR}/config/controller.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/vehicle.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tracks/virtual_track/raceline.csv)

  add_executable(cpp_solver_smoke tests/cpp_solver_smoke.cpp)
  target_compile_features(cpp_solver_smoke PUBLIC cxx_std_17)
  target_link_libraries(cpp_solver_smoke f1tenth_mpcc_acados_runtime)

  add_executable(cpp_delayed_closed_loop tests/cpp_delayed_closed_loop.cpp)
  target_compile_features(cpp_delayed_closed_loop PUBLIC cxx_std_17)
  target_link_libraries(cpp_delayed_closed_loop f1tenth_mpcc_acados_runtime)

  add_executable(cpp_candidate_closed_loop tests/cpp_candidate_closed_loop.cpp)
  target_compile_features(cpp_candidate_closed_loop PUBLIC cxx_std_17)
  target_link_libraries(cpp_candidate_closed_loop f1tenth_mpcc_core)
  add_test(NAME cpp_candidate_closed_loop COMMAND
    $<TARGET_FILE:cpp_candidate_closed_loop>
    ${CMAKE_CURRENT_SOURCE_DIR}/config/controller_stable_v2_racelineV2.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/vehicle_stable_v2_racelineV2.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/data/tracks/racelineV2/raceline.csv)
endif()
