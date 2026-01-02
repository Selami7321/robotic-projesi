# Home Cleaning Robot - ROS2 Autonomous Navigation System

## Overview
This project implements a complete autonomous home cleaning robot using ROS2 (Robot Operating System 2) with Gazebo simulation. The robot is designed to navigate through a home environment, map the space, and perform cleaning tasks using SLAM (Simultaneous Localization and Mapping) and navigation algorithms.

## Features
- **Gazebo Simulation Environment**: Complete home environment with furniture and obstacles
- **SLAM Implementation**: Real-time mapping using `slam_toolbox`
- **Navigation Stack**: Full Nav2 integration for autonomous navigation
- **Robot Control**: Differential drive robot with lidar, camera, and other sensors
- **Coverage Planning**: Systematic cleaning path planning
- **Battery Simulation**: Realistic battery management and charging behavior
- **RViz Integration**: Visualization and control interface

## System Architecture

### Hardware Simulation
- **Differential Drive Robot**: Custom robot model with wheel odometry
- **Sensors**:
  - 2D Lidar for obstacle detection and mapping
  - RGB Camera for visual feedback
  - Depth Camera for 3D perception
- **Actuators**: Wheel controllers for movement

### Software Components
- **ROS2 Foxy/Humble**: Robot Operating System framework
- **Gazebo**: Physics simulation environment
- **SLAM Toolbox**: Online asynchronous SLAM for mapping
- **Navigation2 (Nav2)**: Path planning and navigation
- **Robot State Publisher**: Robot model visualization
- **Twist Multiplexer**: Command arbitration

### Project Structure
```
robot3/
├── src/
│   └── auto_robot/
│       ├── config/          # Configuration files (YAML)
│       ├── description/     # Robot URDF/XACRO models
│       ├── launch/          # ROS2 launch files
│       ├── scripts/         # Python scripts
│       ├── worlds/          # Gazebo world files
│       └── CMakeLists.txt   # Build configuration
├── README.md              # Project documentation
├── setup_env.sh           # Environment setup script
├── setup_project.sh       # Project initialization script
└── DOCKER_INSTRUCTIONS.md # Docker deployment guide
```

## Installation

### Prerequisites
- Ubuntu 20.04 or 22.04
- ROS2 Foxy or Humble
- Gazebo Garden or compatible version
- Python 3.8+

### Setup Steps

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Selami7321/robotic-projesi.git
   cd robotic-projesi
   ```

2. **Install ROS2 dependencies**:
   ```bash
   # Update package lists
   sudo apt update
   
   # Install ROS2 packages
   sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-teleop-twist-keyboard ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-gazebo-ros2-control
   ```

3. **Build the workspace**:
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Go to project directory
   cd /path/to/robotic-projesi
   
   # Build the workspace
   colcon build --packages-select auto_robot
   source install/setup.bash
   ```

4. **Run the simulation**:
   ```bash
   # Launch the simulation
   ros2 launch auto_robot launch_sim.launch.py
   ```

## Running the Simulation

### Launch the Complete System
```bash
# Source the workspace
source install/setup.bash

# Launch the full simulation with navigation
ros2 launch auto_robot launch_sim.launch.py
```

### Launch Individual Components
```bash
# Launch only the robot state publisher
ros2 launch auto_robot rsp.launch.py

# Launch only Gazebo with the world
ros2 launch auto_robot simple_gazebo.launch.py

# Launch only the navigation stack
ros2 launch nav2_bringup navigation_launch.py bt_params_file:=/path/to/your/behavior_tree.xml
```

## Configuration Files

### Navigation Parameters (`config/nav2_params.yaml`)
- Costmap configurations for local and global planners
- Planner and controller settings
- Recovery behavior parameters

### SLAM Parameters (`config/mapper_params_online_async.yaml`)
- SLAM algorithm settings
- Map resolution and size parameters
- Sensor data processing parameters

### Gazebo Parameters (`config/gazebo_params.yaml`)
- Physics engine settings
- Simulation time parameters

### Twist Multiplexer (`config/twist_mux.yaml`)
- Command priority settings
- Topic arbitration rules

## World Description

The simulation includes a detailed home environment:
- Four complete walls forming a closed room
- Living room with sofa, table, TV stand
- Dining area with table and chairs
- Kitchen area with counter and appliances
- Bedrooms with beds and wardrobes
- Proper lighting and decorative elements

## Custom Scripts

### `battery_simulator.py`
- Simulates battery drain based on robot movement
- Publishes battery status to ROS2 topics
- Handles charging behavior when battery is low

### `coverage_planner.py`
- Implements systematic cleaning patterns
- Plans coverage paths for room cleaning
- Integrates with navigation system

### `rviz_cleaning_controls.py`
- RViz-based cleaning control interface
- Allows manual and automatic cleaning mode selection

## Launch Files

### `launch_sim.launch.py`
Main launch file that starts:
- Robot state publisher
- Gazebo simulation with house world
- Robot spawning in the environment
- SLAM toolbox for mapping
- Full Nav2 navigation stack
- Robot behavior manager
- Coverage planner
- Simple navigation script
- Battery simulator

### `rsp.launch.py`
Launches the robot state publisher for model visualization.

### `simple_gazebo.launch.py`
Launches Gazebo with the world file without additional ROS nodes.

## Development Guidelines

### Code Structure
- Use ROS2 packages for modularity
- Follow ROS2 naming conventions
- Implement proper error handling
- Use parameter servers for configuration

### Testing
- Test individual components before integration
- Use RViz for visualization debugging
- Monitor TF trees for transform issues
- Check topic connections and message rates

## Troubleshooting

### Common Issues
1. **Robot not spawning**: Check Gazebo connection and model description
2. **Navigation not working**: Verify costmaps and local/global planners
3. **SLAM not building map**: Check lidar data and SLAM parameters
4. **TF issues**: Verify all coordinate frames are properly published

### Debugging Commands
```bash
# Check all topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Echo a specific topic
ros2 topic echo /topic_name std_msgs/msg/String
```

## Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Authors
- Selami Çetin - Project Developer
- Uğur Baki Arslan - Project Developer

## Acknowledgments
- ROS2 and Navigation2 communities
- Gazebo simulation framework
- SLAM Toolbox developers
