# Joined Positioning and Control (JPC) Project

## Overview

This project extends the Fast-Drone-250 repository (original project: [ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250.git)) by implementing a real-time Model Predictive Control (MPC) based on Factor Graph Optimization (FGO). The system provides advanced estimation and control capabilities for quadrotor drones with enhanced safety features through Control Barrier Function (CBF) based obstacle avoidance.

## Key Features

- **Real-time MPC with Factor Graph Optimization**: Advanced trajectory optimization and control
- **CBF-based Obstacle Avoidance**: Safety-critical control with formal collision avoidance guarantees
- **Multi-threaded Data Processing**: AsyncSpinner architecture for reduced latency and improved real-time performance
- **Thread-safe Data Management**: Mutex-protected data structures for robust concurrent access

## Installation and Setup

### Prerequisites
- ROS (tested with [your ROS version])
- GTSAM 4.0.3 (Georgia Tech Smoothing and Mapping library)
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
Launch the MPC controller with CBF obstacle avoidance:
```bash
roslaunch jpcm run_ctrl_mpc_vicon.launch
```

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

2. **Dynamic Obstacle Handling**:
   - Real-time obstacle position and velocity tracking
   - Adaptive safety margins based on relative velocity
   - Multi-obstacle support with individual safety constraints

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

3. **Joined Positioning and Control Model**:
   - Tightly coupled estimation and control framework
   - Real-time feedback between perception and action
   - Experimental advanced features in active development

## Performance Improvements

### Latency Reduction
- **Multi-threaded Processing**: AsyncSpinner eliminates callback bottlenecks
- **Optimized Data Flow**: Reduced delay between sensor input and control output
- **Real-time Responsiveness**: Improved performance in dynamic environments

### Robustness Enhancements
- **Mutex Protection**: Prevents data race conditions in concurrent access
- **Graceful Degradation**: Maintains functionality under heavy computational load
- **Error Handling**: Comprehensive exception handling and recovery mechanisms

## Dependencies

- **GTSAM**: Version 4.0.3 required
  - Installation guide: [GTSAM GitHub](https://github.com/borglab/gtsam)
- **ROS Packages**: mavros, vrpn_client_node, and standard perception stack

## Usage Examples

### Basic Operation
```bash
# Start motion capture system
roslaunch vrpn_client_node motion_capture.launch

# Launch the enhanced MPC controller
roslaunch jpcm run_ctrl_mpc_vicon.launch

# Monitor obstacle avoidance performance
rostopic echo /debug/obstacle_info
```

### Advanced Configuration
Modify CBF parameters in `config/obstacle_avoidance.yaml`:
```yaml
cbf_parameters:
  safety_margin: 0.3      # Minimum safe distance (meters)
  alpha: 2.0              # CBF relaxation parameter
  max_avoidance_force: 5.0 # Maximum avoidance control authority
```

## Troubleshooting

### Common Issues
1. **GTSAM Version Compatibility**:
   ```bash
   # Ensure correct GTSAM version
   git clone https://github.com/borglab/gtsam.git
   cd gtsam && git checkout 4.0.3
   ```

2. **ROS Dependency Resolution**:
   ```bash
   rosdep install --from-paths src --ignore-src -y
   ```

3. **Real-time Performance**:
   - Ensure proper thread prioritization
   - Monitor CPU usage with `top` or `htop`
   - Adjust AsyncSpinner thread count based on available cores

### Performance Monitoring
```bash
# Monitor system latency
rostopic hz /mavros/odometry/in

# Check thread performance
top -H -p $(pgrep -f jpcm_node)
```

## License

This project builds upon the Fast-Drone-250 codebase from ZJU-FAST-Lab. Please refer to the original project for licensing details. Additional implementations are provided under [Your License Choice].

## Acknowledgements

This project builds upon the outstanding work from **ZJU-FAST-Lab's Fast-Drone-250**. We gratefully acknowledge their significant contribution to the open-source community and their pioneering work in quadrotor control systems.

### Special Thanks
- ZJU-FAST-Lab for the original Fast-Drone-250 framework
- Georgia Tech for the GTSAM optimization library
- ROS community for the extensive robotics middleware ecosystem

## Citation

If you use this work in your research, please consider citing:
```bibtex
@software{jpc_project_2024,
  title = {Joined Positioning and Control with CBF Obstacle Avoidance},
  author = {Your Name and Contributors},
  year = {2024},
  url = {https://github.com/your-repo/jpcm},
  note = {Extension of Fast-Drone-250 with safety-critical features}
}
```

## Future Work

- [ ] 3D obstacle representation and avoidance
- [ ] Multi-agent collision avoidance
- [ ] Adaptive CBF parameters based on environment complexity
- [ ] Hardware-in-the-loop validation
- [ ] Extended sensor fusion (LiDAR, depth cameras)

---

For questions and support, please open an issue on our GitHub repository or contact the development team.