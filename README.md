# Joined Positioning and Control (JPC) Project

### Issues

* When the CPU is too complex and heavy, it may cause ROS lag, causing controller jitter, and even exiting the flight.
* Tunning Method - Pending

## Overview

This project extends the Fast-Drone-250 repository (original project: [ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250.git)) by implementing a real-time Model Predictive Control (MPC) based on Factor Graph Optimization (FGO). The system provides advanced estimation and control capabilities for quadrotor drones with enhanced safety features through Control Barrier Function (CBF) based obstacle avoidance.

## Key Features

- **Real-time MPC with Factor Graph Optimization**: Advanced trajectory optimization and control
- **CBF-based Obstacle Avoidance**: Safety-critical control with formal collision avoidance guarantees
- **Multi-threaded Data Processing**: AsyncSpinner architecture for reduced latency and improved real-time performance
- **Thread-safe Data Management**: Mutex-protected data structures for robust concurrent access

## Installation and Setup

### Prerequisites

- ROS (tested with [noetic])
- **GTSAM 4.0.3 **(Georgia Tech Smoothing and Mapping library)
- mavros
- vrpn_client_node

### Getting Started

1. **Copy necessary files**:
   
   ```bash
   cp -r src/util/ [your_workspace_path]/src/
   ```
2. **Build the workspace**:
   
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Running the System

### Motion Capture Setup

To obtain quadrotor motion data:

```bash
roslaunch vrpn_client_node ***.launch
roslaunch jpcm vicon.launch
```

### Main Controller

```bash
roslaunch jpcm run_ctrl_mpc_vicon.launch
```

#### Or Launch the MPC controller with CBF obstacle avoidance:

```bash
roslaunch jpcm run_ctrl_factor_mpc_vicon.launch
```


### Takeof Command

```rostopic pub -1  /takeoff\_land quadrotor\_msgs/TakeoffLand "takeoff\_land\_cmd: 1” ```

### Trajectory Sender Example

https://github.com/RoboticsPolyu/IPN_MPC/blob/JPCM-controller/app/Traj_load_run_px4ctrl.cpp

### Test Program

Run the test suite:

```bash
roslaunch jpcm run_test.launch
```

## System Architecture

### Enhanced Data Processing

- **AsyncSpinner Implementation**: Multi-threaded callback handling for reduced data latency
- **Thread-safe Data Structures**: Mutex-protected access to odometry, IMU, and obstacle data
- **Real-time Performance**: Optimized data flow between perception, estimation, and control modules

### Safety Features

1. **CBF-based Obstacle Avoidance Factors**:
   
   - Formal safety guarantees for collision avoidance
   - Real-time barrier function evaluation
   - Integration with FGO optimization framework

### Estimator Modules

1. **FGO-based FakeGPS + IMU**:
   - Estimates pose, velocity, bias, and gravity rotation
   - Combines visual and inertial data for robust state estimation
   - Thread-safe data fusion with mutex protection

### Controller Modules

1. **FGO-based MPC with CBF Constraints**:
   
   - Model Predictive Control using Factor Graph Optimization
   - Integrated CBF constraints for obstacle avoidance
   - Real-time optimal trajectory generation with safety guarantees
2. **Uncertainty-aware MPC**:
   
   - Accounts for estimation uncertainties in control decisions
   - More robust performance under noisy conditions
   - Adaptive control authority based on confidence levels
3. **Differential-Flatness-Based Control (PID)**👍

- Traditional PID controller with differential flatness transformation

4. **Joined Positioning and Control Model** (Experimental):

### Advanced Configuration

Modify CBF parameters in `config/obstacle_avoidance.yaml`:

```yaml
CBF_alpha:          0.1     
CBF_beta:           0.1     
point_obs_sigma:    0.1     
quad_radius:        0.20      
safe_d:             0.05
```


## License

This project builds upon the Fast-Drone-250 codebase from ZJU-FAST-Lab. Please refer to the original project for licensing details. Additional implementations are provided under [Your License Choice].


### Special Thanks

- ZJU-FAST-Lab for the original Fast-Drone-250 framework
- Georgia Tech for the GTSAM optimization library
- ROS community for the extensive robotics middleware ecosystem

## Citation

If you use this work in your research, please consider citing:

```bibtex
@ARTICLE{11082016,
  author={Yang, Peiwen and Wen, Weisong and Bai, Shiyu and Hsu, Li-Ta},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={Tightly Joined Positioning and Control Model for Unmanned Aerial Vehicles Based on Factor Graph Optimization}, 
  year={2025},
  volume={},
  number={},
  pages={1-15},
  keywords={Uncertainty;Autonomous aerial vehicles;Vehicle dynamics;Aerodynamics;Global navigation satellite system;Trajectory;Cost function;Motion control;Covariance matrices;Pipelines;Positioning;Model predictive control (MPC);Dynamic model;Factor graph optimization (FGO);Joint optimization;Positioning uncertainty, Unmanned aerial vehicles (UAV)},
  doi={10.1109/TVT.2025.3589556}}
```

## Future Work

- [ ] 3D obstacle representation and avoidance
- [ ] Multi-agent collision avoidance
- [ ] Adaptive CBF parameters based on environment complexity
- [ ] Hardware-in-the-loop validation
- [ ] Extended sensor fusion (LiDAR, depth cameras)

For questions and support, please open an issue on our GitHub repository or contact the development team.

peiwen1.yang@connect.polyu.hk


