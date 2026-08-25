cmake_minimum_required(VERSION 3.0.2)
project(raw_cloud_obstacle_perception)

add_compile_options(-std=c++14 -O3 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

catkin_package(CATKIN_DEPENDS
  geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs tf2_ros
  visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(raw_cloud_obstacle_node src/raw_cloud_obstacle_node.cpp)
target_link_libraries(raw_cloud_obstacle_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS raw_cloud_obstacle_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/start_raw_cloud_obstacle.sh
  scripts/stop_raw_cloud_obstacle.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
