cmake_minimum_required(VERSION 2.8.3)
project(jpcm)

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

# Create GTSAMIntegration library
add_library(gtsam_integration_lib
  GTSAMIntegration/CBF_factor.cpp
  GTSAMIntegration/factors.cpp
  GTSAMIntegration/GTSAMUtils.cpp
  GTSAMIntegration/JPCM.cpp
  GTSAMIntegration/Marginalization.cpp
)

target_link_libraries(gtsam_integration_lib
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

# Add dependencies for the library
add_dependencies(gtsam_integration_lib quadrotor_msgs geometry_msgs)

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

# JPCM_node using the library
add_executable(JPCM_node 
  app/JPCM_node.cpp
  src/controller.cpp
  src/input.cpp  
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
)

add_dependencies(JPCM_node quadrotor_msgs)

target_link_libraries(JPCM_node
  gtsam_integration_lib
  ${catkin_LIBRARIES}
  gtsam 
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# test_jpcm using the library
add_executable(test_jpcm   
  src/controller.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
  test/test_jpcm.cpp
)

add_dependencies(test_jpcm quadrotor_msgs geometry_msgs)

target_link_libraries(test_jpcm
  gtsam_integration_lib
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

# FakeGPS_IMU_fusion_node using the library
add_executable(FakeGPS_IMU_fusion_node
  app/FakeGPS_IMU_fusion_node.cpp  
  src/controller.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
)

add_dependencies(FakeGPS_IMU_fusion_node quadrotor_msgs geometry_msgs)

target_link_libraries(FakeGPS_IMU_fusion_node
  gtsam_integration_lib
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)

# FactorMPC_node using the library
add_executable(FactorMPC_node 
  app/FactorMPC_node.cpp
  src/controller.cpp
  src/input.cpp  
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
)

add_dependencies(FactorMPC_node quadrotor_msgs)

target_link_libraries(FactorMPC_node
  gtsam_integration_lib
  ${catkin_LIBRARIES}
  gtsam 
  gtsam_unstable
  ${YAML_CPP_LIBRARIES}
)