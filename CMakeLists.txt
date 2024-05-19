cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 
find_package(GTSAM 4 REQUIRED)
find_package(yaml-cpp)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include/${PROJECT_NAME}
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

include_directories("/home/amov/Fast250/devel/include")

add_executable(odom_node        
  src/odom_node.cpp
)

add_dependencies(odom_node quadrotor_msgs geometry_msgs)

target_link_libraries(odom_node
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node quadrotor_msgs)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  gtsam 
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
