cmake_minimum_required(VERSION 3.0.2)
project(smc_quadrotor_cpp)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  mav_msgs
  tf
)

## Find Eigen3
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smc_quadrotor_lib
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs mav_msgs tf
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library for trajectory generator and SMC controller
add_library(smc_quadrotor_lib
  src/trajectory_generator.cpp
  src/smc_controller.cpp
)

## Add cmake target dependencies of the library
add_dependencies(smc_quadrotor_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(smc_quadrotor_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the main quadrotor control node
add_executable(quadrotor_control_node src/quadrotor_control.cpp)

## Add cmake target dependencies of the executable
add_dependencies(quadrotor_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link
target_link_libraries(quadrotor_control_node
  smc_quadrotor_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS quadrotor_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS smc_quadrotor_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
