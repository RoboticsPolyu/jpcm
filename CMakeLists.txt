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
  GTSAMIntegration
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(gps_imu        
  src/gps_imu.cpp
  app/gps_imu_node.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
)

add_dependencies(gps_imu quadrotor_msgs geometry_msgs)

target_link_libraries(gps_imu
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

add_executable(odom_node        
  app/odom_node.cpp
  src/PX4CtrlParam.cpp
)

add_dependencies(odom_node quadrotor_msgs geometry_msgs)

target_link_libraries(odom_node
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

add_executable(px4ctrl_node 
  app/px4ctrl_node.cpp
  src/controller.cpp
  src/input.cpp  
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  GTSAMIntegration/factors.cpp
  GTSAMIntegration/GTSAMUtils.cpp
  GTSAMIntegration/JPCM.cpp
  GTSAMIntegration/Marginalization.cpp
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

add_executable(test_jpcm        
  GTSAMIntegration/factors.cpp
  GTSAMIntegration/GTSAMUtils.cpp
  GTSAMIntegration/JPCM.cpp
  GTSAMIntegration/Marginalization.cpp
  src/controller.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
  test/test_jpcm.cpp
)

add_dependencies(test_jpcm quadrotor_msgs geometry_msgs)

target_link_libraries(test_jpcm
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

add_executable(test_fusion  
  GTSAMIntegration/factors.cpp
  GTSAMIntegration/GTSAMUtils.cpp
  GTSAMIntegration/JPCM.cpp
  GTSAMIntegration/Marginalization.cpp    
  test/test_fusion.cpp
  src/controller.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
)

add_dependencies(test_fusion quadrotor_msgs geometry_msgs)

target_link_libraries(test_fusion
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)