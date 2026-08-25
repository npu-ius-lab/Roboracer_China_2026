cmake_minimum_required(VERSION 3.10)
project(point_lio_sam_lighterbev LANGUAGES CXX)
set(POINT_LIO_SAM_WORKSPACE_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# --------------------------
# Build type & C++ standard
# --------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# libtorch 2.x requires C++17, while the vehicle's libtorch 1.12 headers must
# be compiled as C++14 (their c10::optional clashes with std::optional in 17).
set(POINT_LIO_SAM_CXX_STANDARD "17" CACHE STRING
    "C++ standard for point_lio_sam_lighterbev (14 for libtorch 1.x, 17 for 2.x)")
set(CMAKE_CXX_STANDARD ${POINT_LIO_SAM_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 常用编译/链接选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS_RELEASE}")

# --------------------------
# OpenMP
# --------------------------
include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "OpenMP not found; continuing without OpenMP.")
endif()

# --------------------------
# Dependencies
# --------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  message_filters
  nano_gicp
  cv_bridge            # <--- 新增
  image_transport      # <--- 建议顺带加上
  message_generation
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

# Torch (libtorch)
set(Torch_DIR "" CACHE PATH "Path to the libtorch CMake package directory (the folder that contains TorchConfig.cmake)")
if(Torch_DIR)
  list(APPEND CMAKE_PREFIX_PATH "${Torch_DIR}")
elseif(DEFINED ENV{Torch_DIR} AND NOT "$ENV{Torch_DIR}" STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{Torch_DIR}")
endif()
find_package(Torch REQUIRED)

# 让所有目标都继承 Torch 的编译选项（如 _GLIBCXX_USE_CXX11_ABI）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_message_files(
  FILES
  BevFrame.msg
  LocalizationStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)


# --------------------------
# catkin package
# --------------------------
catkin_package(
  CATKIN_DEPENDS
    roscpp roslib rosbag std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
    visualization_msgs tf tf_conversions pcl_ros pcl_conversions
    message_filters nano_gicp
    cv_bridge image_transport     # <--- 新增
)

# --------------------------
# Include directories
# --------------------------
include_directories(
  include
  ${POINT_LIO_SAM_WORKSPACE_SRC_DIR}/third_party/LighterBEV
  # nano_gicp.hpp exposes its incremental target API with ikd_Tree.h in the
  # public header, so consumers must see this directory even when an older
  # catkin overlay has a stale nano_gicpConfig.cmake.
  ${POINT_LIO_SAM_WORKSPACE_SRC_DIR}/third_party/ikd-tree
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

# --------------------------
# External subprojects
# --------------------------
# LighterBEV
# ⚠️ 请在 third_party/LighterBEV/CMakeLists.txt 中将
# add_library(LighterBEVManager MODULE …)
# 改为：add_library(LighterBEVManager STATIC …)
set(LighterBEV_SOURCE_DIR ${POINT_LIO_SAM_WORKSPACE_SRC_DIR}/third_party/LighterBEV)
set(LighterBEV_BINARY_DIR ${CMAKE_BINARY_DIR}/third_party/LighterBEV)
add_subdirectory(${LighterBEV_SOURCE_DIR} ${LighterBEV_BINARY_DIR})

# --------------------------
# Targets
# --------------------------
set(COMMON_LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  gtsam
)

add_library(${PROJECT_NAME}_modules SHARED
  src/loop_closure.cpp
  src/point_lio_sam_lighterbev.cpp
  src/robust_localizer.cpp
)
set_target_properties(${PROJECT_NAME}_modules PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME}_modules
  ${COMMON_LIBS}
  ${TORCH_LIBRARIES}
  LighterBEVManager
)
add_dependencies(${PROJECT_NAME}_modules
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_modules
  ${COMMON_LIBS}
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(pointlio_robust_localization_node
  src/robust_localization_main.cpp
)
target_link_libraries(pointlio_robust_localization_node
  ${PROJECT_NAME}_modules
  ${COMMON_LIBS}
)
add_dependencies(pointlio_robust_localization_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# Keep runtime assets next to the installed ROS package so launch files remain
# relocatable after catkin_make install or deployment to another machine.
install(TARGETS
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_node
  pointlio_robust_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/save_map_relative.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY point_lio/localization_map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/point_lio
)

# --------------------------
# Diagnostics (optional)
# --------------------------
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Torch include: ${TORCH_INCLUDE_DIRS}")
message(STATUS "Torch libs   : ${TORCH_LIBRARIES}")
