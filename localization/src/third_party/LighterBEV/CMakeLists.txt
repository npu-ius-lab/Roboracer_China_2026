cmake_minimum_required(VERSION 3.10)
project(LighterBEV LANGUAGES CXX)

# -------------------------
# C++ 标准
# -------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 可选：构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# -------------------------
# 依赖
# -------------------------
# The parent point_lio_sam_lighterbev CMakeLists already finds Torch before
# add_subdirectory(). Re-running find_package(Torch) here re-includes
# Caffe2's mkl.cmake in a child scope, where target_link_libraries on the
# parent-scope caffe2::mkl imported target fails with
# "not built by this project". Reuse the parent target when present.
if(NOT TARGET torch)
  find_package(Torch REQUIRED)
endif()
find_package(OpenCV REQUIRED)
# Eigen 仅头文件依赖，如果需要可启用
find_package(Eigen3 QUIET)

# 如果子目录单独构建（非从 catkin 顶层进来），尝试静默找 catkin
find_package(catkin QUIET COMPONENTS roscpp)

# 透传 libtorch 的编译选项（包含 _GLIBCXX_USE_CXX11_ABI 等）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# -------------------------
# 目标：LighterBEVManager
# -------------------------
# 可切换为 SHARED（默认 STATIC 更稳）
option(LIGHTERBEV_BUILD_SHARED "Build LighterBEVManager as SHARED library" OFF)
set(LIGHTERBEV_MANAGER_SOURCES
  Lighterbev.cpp
)

if(LIGHTERBEV_BUILD_SHARED)
  add_library(LighterBEVManager SHARED ${LIGHTERBEV_MANAGER_SOURCES})
else()
  add_library(LighterBEVManager STATIC ${LIGHTERBEV_MANAGER_SOURCES})
endif()

# 保证可被其它共享库链接
set_target_properties(LighterBEVManager PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# 公开头文件路径
# 本目录包含：Lighterbev.h、bev_frame.h、nanoflann.hpp、KDTreeVectorOfVectorsAdaptor.h 等
target_include_directories(LighterBEVManager
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../point_lio_sam_lighterbev/include
    ${OpenCV_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
)

# 如果 Eigen3 可用，公开其头文件
if(Eigen3_FOUND)
  target_include_directories(LighterBEVManager PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

# 如果从 catkin 顶层进入，公开 catkin 的头文件（可选）
if(catkin_FOUND)
  target_include_directories(LighterBEVManager PUBLIC ${catkin_INCLUDE_DIRS})
endif()

# 链接依赖
target_link_libraries(LighterBEVManager
  PUBLIC
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
)

# 如存在 catkin 库，链接之（可选；用于 ROS 日志等）
if(catkin_FOUND)
  target_link_libraries(LighterBEVManager PUBLIC ${catkin_LIBRARIES})
endif()

# -------------------------
# 编译/链接优化（可选）
# -------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(LighterBEVManager PRIVATE -O3 -Wall -Wextra)
else()
  target_compile_options(LighterBEVManager PRIVATE -Wall -Wextra)
endif()

# -------------------------
# 诊断信息（可选）
# -------------------------
message(STATUS "[LighterBEV] Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "[LighterBEV] OpenCV inc : ${OpenCV_INCLUDE_DIRS}")
message(STATUS "[LighterBEV] Torch inc  : ${TORCH_INCLUDE_DIRS}")
message(STATUS "[LighterBEV] Torch libs : ${TORCH_LIBRARIES}")
