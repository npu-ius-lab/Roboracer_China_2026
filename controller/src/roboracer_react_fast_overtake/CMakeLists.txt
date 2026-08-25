cmake_minimum_required(VERSION 3.0.2)
project(roboracer_react_fast_overtake)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboracer_react_fast_core
  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp sensor_msgs std_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(roboracer_react_fast_core src/react_core.cpp)
target_compile_features(roboracer_react_fast_core PUBLIC cxx_std_17)
target_link_libraries(roboracer_react_fast_core ${catkin_LIBRARIES})

add_library(roboracer_react_fast_minimum_speed_gate_core src/minimum_speed_gate.cpp)
target_compile_features(roboracer_react_fast_minimum_speed_gate_core PUBLIC cxx_std_17)
target_link_libraries(roboracer_react_fast_minimum_speed_gate_core ${catkin_LIBRARIES})

add_executable(roboracer_react_fast_overtake_node src/react_overtake_node.cpp)
target_compile_features(roboracer_react_fast_overtake_node PUBLIC cxx_std_17)
target_link_libraries(roboracer_react_fast_overtake_node
  roboracer_react_fast_core ${catkin_LIBRARIES})

add_executable(roboracer_react_fast_minimum_speed_gate_node
  src/minimum_speed_gate_node.cpp)
target_compile_features(roboracer_react_fast_minimum_speed_gate_node PUBLIC cxx_std_17)
target_link_libraries(roboracer_react_fast_minimum_speed_gate_node
  roboracer_react_fast_minimum_speed_gate_core ${catkin_LIBRARIES})

install(TARGETS
  roboracer_react_fast_core
  roboracer_react_fast_minimum_speed_gate_core
  roboracer_react_fast_overtake_node
  roboracer_react_fast_minimum_speed_gate_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch tracks
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(roboracer_react_fast_core_test test/react_core_test.cpp)
  if(TARGET roboracer_react_fast_core_test)
    target_compile_features(roboracer_react_fast_core_test PUBLIC cxx_std_17)
    target_link_libraries(roboracer_react_fast_core_test
      roboracer_react_fast_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(roboracer_react_fast_minimum_speed_gate_test
    test/minimum_speed_gate_test.cpp)
  if(TARGET roboracer_react_fast_minimum_speed_gate_test)
    target_compile_features(roboracer_react_fast_minimum_speed_gate_test PUBLIC cxx_std_17)
    target_link_libraries(roboracer_react_fast_minimum_speed_gate_test
      roboracer_react_fast_minimum_speed_gate_core ${catkin_LIBRARIES})
  endif()
endif()
