# Joined Positioning and Control (JPC) Project

### Issues

* When the CPU is too complex and heavy, it may cause ROS lag, causing controller jitter, and even exiting the flight.

## Overview

This project extends the Fast-Drone-250 repository (original project: [ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250.git)) by implementing a real-time Model Predictive Control (MPC) based on Factor Graph Optimization (FGO). The system provides advanced estimation and control capabilities for quadrotor drones.

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

Launch the MPC controller:

```bash
roslaunch jpcm run_ctrl_mpc_vicon.launch
```

### Takeof Command


```rostopic pub -1  /takeoff\_land quadrotor\_msgs/TakeoffLand "takeoff\_land\_cmd: 1”``

### Trajectory Sender Example

https://github.com/RoboticsPolyu/IPN_MPC/blob/JPCM-controller/app/Traj_load_run_px4ctrl.cpp

### Test Program

Run the test suite:

```bash
roslaunch jpcm run_test.launch
```

## System Components

### Estimator Modules

1. **FGO-based FakeGPS + IMU**:
   - Estimates pose, velocity, bias, and gravity rotation
   - Combines visual and inertial data for robust state estimation

### Controller Modules

1. **Differential-Flatness-Based Control (PID)**:
   
   - Traditional PID controller with differential flatness transformation
2. **FGO-based MPC**:
   
   - Model Predictive Control using Factor Graph Optimization
   - Real-time optimal trajectory generation
3. **Uncertainty-aware MPC**:
   
   - Accounts for estimation uncertainties in control decisions
   - More robust performance under noisy conditions
4. **Joined Positioning and Control Model** (Experimental):
   
   - Tightly coupled estimation and control framework
   - Currently in testing phase

## Dependencies

- **GTSAM**: Version 4.0.3 required
  - Installation guide: [GTSAM GitHub](https://github.com/borglab/gtsam)

## Troubleshooting

[Add any common issues and solutions here]

## License

[Specify your license here, or note if it inherits from the original project]

## Acknowledgements

This project builds upon the work from ZJU-FAST-Lab's Fast-Drone-250. We gratefully acknowledge their contribution to the open-source community.

