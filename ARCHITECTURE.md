# Home Cleaning Robot - System Architecture Documentation

## Overview
This document describes the architecture of the autonomous home cleaning robot system built using ROS2 and Gazebo simulation. The system integrates multiple robotics technologies including SLAM, navigation, and task planning.

## System Components

### 1. Robot Model (URDF/XACRO)
The robot is defined using Unified Robot Description Format (URDF) with XACRO macros for modularity:
- **Base**: Differential drive platform with wheel odometry
- **Sensors**:
  - 2D Lidar (LDS-01 compatible) for obstacle detection and mapping
  - RGB Camera for visual feedback
  - Depth Camera for 3D perception
  - IMU for orientation data
- **Actuators**: Wheel controllers for movement

### 2. Simulation Environment (Gazebo)
The simulation environment provides:
- Physics engine for realistic robot dynamics
- Sensor simulation with noise models
- Visual rendering of the home environment
- Ground truth for testing and validation

### 3. Robot State Publisher
- Publishes robot joint states to TF tree
- Maintains coordinate frame relationships
- Provides robot model visualization in RViz

### 4. SLAM System (SLAM Toolbox)
The Simultaneous Localization and Mapping system:
- Creates real-time map of the environment
- Estimates robot pose relative to the map
- Handles loop closure and map optimization
- Publishes occupancy grid map for navigation

### 5. Navigation System (Navigation2)
The navigation stack includes:
- **Global Planner**: A*, NavFn, or other path planning algorithms
- **Local Planner**: Trajectory rollout for obstacle avoidance
- **Controller**: Follow trajectory action server
- **Costmap**: Static and obstacle layer management
- **Behavior Tree**: Task execution and recovery behaviors

### 6. Task Planning and Execution
- **Coverage Planner**: Systematic cleaning path planning
- **Behavior Manager**: High-level task orchestration
- **Battery Manager**: Energy-aware navigation and charging

## Software Architecture

### ROS2 Nodes

#### Core Nodes
- `robot_state_publisher`: Publishes robot model transforms
- `gazebo_ros`: Gazebo simulation interface
- `slam_toolbox`: SLAM and mapping
- `nav2_bringup`: Navigation stack
- `twist_mux`: Command multiplexing

#### Custom Nodes
- `battery_simulator.py`: Battery level simulation
- `coverage_planner.py`: Cleaning path planning
- `simple_auto_nav.py`: Simple navigation controller
- `robot_behavior_manager.py`: Task management

### Message Types
- `sensor_msgs/LaserScan`: Lidar data
- `sensor_msgs/Image`: Camera data
- `geometry_msgs/Twist`: Velocity commands
- `nav_msgs/OccupancyGrid`: Map data
- `nav_msgs/Path`: Planned paths
- `geometry_msgs/PoseStamped`: Goal poses
- `std_msgs/Float32`: Battery level

### Action Servers
- `nav2_msgs.NavigateToPose`: Navigation goals
- `nav2_msgs.ComputePathToPose`: Path planning
- Custom action servers for cleaning tasks

### Topics
- `/cmd_vel`: Velocity commands
- `/scan`: Laser scan data
- `/camera/image_raw`: Camera images
- `/map`: Occupancy grid map
- `/tf` and `/tf_static`: Transform data
- `/goal_pose`: Navigation goals
- `/battery_status`: Battery level

## Launch System

### Main Launch File (`launch_sim.launch.py`)
This is the primary launch file that starts the complete system:
1. Robot State Publisher
2. Gazebo with house world
3. Robot spawning
4. SLAM Toolbox
5. Navigation2 stack
6. Custom behavior nodes
7. Coverage planner
8. Battery simulator

### Component Launch Files
- `rsp.launch.py`: Robot State Publisher only
- `simple_gazebo.launch.py`: Gazebo simulation only
- Specialized launch files for testing individual components

## Configuration Files

### Navigation Parameters (`config/nav2_params.yaml`)
- Local and global costmap configurations
- Planner and controller parameters
- Recovery behavior settings
- Transform tolerance and timeout values

### SLAM Parameters (`config/mapper_params_online_async.yaml`)
- SLAM algorithm selection (async online SLAM)
- Map resolution and size
- Sensor data processing parameters
- Loop closure settings

### Robot Description (`description/robot.urdf.xacro`)
- Complete robot model definition
- Sensor and actuator configurations
- Inertial properties
- Visual and collision models

### World Description (`worlds/house.world`)
- Home environment model
- Furniture and obstacles
- Lighting configuration
- Physics properties

## Control Flow

### Initialization
1. Launch files initialize all required nodes
2. Robot model is loaded and transforms are published
3. Gazebo simulation starts with the world
4. Robot is spawned in the environment
5. SLAM and navigation systems initialize

### Operation
1. Robot receives cleaning goals
2. Coverage planner generates systematic cleaning path
3. Navigation system plans and executes paths
4. SLAM continuously updates the map
5. Sensors provide feedback for obstacle avoidance
6. Battery manager monitors energy levels

### Safety
- Collision avoidance through local planner
- Emergency stops when battery is low
- Recovery behaviors for stuck robot
- Transform timeout protection

## Hardware Abstraction

### Simulation vs Real Robot
The system is designed with hardware abstraction in mind:
- Sensor and actuator interfaces through ROS2 drivers
- Configuration files for different robot models
- Simulation-specific parameters in separate files

### Sensor Integration
- Lidar provides 2D environment data
- Camera for visual feedback and object recognition
- IMU for orientation (simulated)
- Wheel encoders for odometry

## Performance Considerations

### Real-time Requirements
- Control loop rates maintained for stable navigation
- Sensor data processing optimized
- Path planning frequency balanced with quality

### Resource Management
- Efficient map representation
- Memory management for long-term operation
- Battery-aware task scheduling

## Testing and Validation

### Simulation Testing
- Unit testing of individual components
- Integration testing of complete system
- Performance testing with different environments

### Metrics
- Map quality and completeness
- Navigation success rate
- Path efficiency
- Battery consumption patterns

## Deployment Instructions

### Local Development
1. Install ROS2 (Humble Hawksbill recommended)
2. Install Gazebo Garden or compatible version
3. Install navigation and SLAM packages
4. Build the workspace with colcon
5. Source the setup files
6. Launch the system

### Docker Deployment
Docker configuration available for containerized deployment:
- Complete ROS2 and Gazebo environment
- Pre-built workspace
- Easy deployment across systems

## Security Considerations

### Data Protection
- Simulation data does not contain real-world information
- No sensitive data transmitted over networks
- Local operation ensures privacy

### Access Control
- Local system access required
- No external communication in basic operation
- Secure authentication for remote operation

## Future Enhancements

### Planned Features
- Object recognition for furniture identification
- Multi-room navigation with door handling
- Advanced cleaning patterns
- Human-robot interaction interfaces
- Fleet management for multiple robots

### Architecture Improvements
- Modular component design
- Improved error handling
- Enhanced localization accuracy
- Better energy management

## Troubleshooting

### Common Issues
1. **TF Tree Issues**: Verify all transforms are being published
2. **Navigation Failures**: Check costmap configurations and sensor data
3. **SLAM Drift**: Validate sensor calibration and parameters
4. **Performance**: Monitor CPU and memory usage

### Debugging Tools
- RViz for visualization
- rqt tools for topic monitoring
- TF view for transform debugging
- Navigation tools for path planning analysis

## References
- ROS2 Documentation
- Navigation2 Tutorials
- SLAM Toolbox Guide
- Gazebo Simulation Documentation