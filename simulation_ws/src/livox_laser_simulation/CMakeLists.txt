cmake_minimum_required(VERSION 3.0.2)
project(livox_laser_simulation)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver
  roscpp
  sensor_msgs
  tf
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES livox_laser_simulation
  CATKIN_DEPENDS livox_ros_driver roscpp sensor_msgs tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
add_dependencies(livox_laser_simulation ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin)

install(TARGETS livox_laser_simulation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY scan_mode
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
