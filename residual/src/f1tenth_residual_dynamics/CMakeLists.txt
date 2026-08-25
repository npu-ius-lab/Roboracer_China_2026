cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_residual_dynamics)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES f1tenth_residual_model
  DEPENDS EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_library(f1tenth_residual_model src/residual_model.cpp)
target_link_libraries(f1tenth_residual_model yaml-cpp ${catkin_LIBRARIES})
add_executable(residual_model_inspect src/residual_model_inspect.cpp)
target_link_libraries(residual_model_inspect f1tenth_residual_model)

catkin_install_python(PROGRAMS
  tools/extract_bag.py
  tools/train_residual.py
  tools/evaluate_residual.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS f1tenth_residual_model residual_model_inspect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
