# HOME CLEANER BOT - COMPLETE IMPLEMENTATION

## Project Overview
This is a fully implemented autonomous robot system for indoor cleaning with obstacle avoidance. The system includes:

1. **2+1 House Layout Model** - Two bedrooms and a living room with realistic furniture placement
2. **SLAM Mapping** - Simultaneous Localization and Mapping using slam_toolbox
3. **Navigation System** - Nav2 with waypoint following for coverage planning
4. **Robot Behaviors** - Mapping, cleaning, and docking modes
5. **Visualization** - RViz controls and real-time monitoring
6. **Docker Containerization** - For easy deployment

## System Components

### House Environment (worlds/house.world)
- 2+1 layout with two bedrooms and living room
- Realistic walls, doors, and furniture
- SDF format for Gazebo simulation

### Robot Hardware
- Differential drive base for mobility
- LiDAR sensor for 360° obstacle detection
- Camera for visual perception
- Depth camera for 3D spatial awareness

### Software Architecture
- **ROS2 Humble** - Core framework
- **Gazebo** - Physics simulation environment
- **SLAM Toolbox** - Mapping and localization
- **Navigation2** - Path planning and obstacle avoidance
- **Python Scripts** - Custom behavior management

## Key Features Implemented

### 1. Intelligent Navigation Algorithm
File: src/auto_robot/scripts/simple_auto_nav.py
- Sector-based obstacle detection using LiDAR
- State machine approach (FORWARD, TURN_LEFT, TURN_RIGHT)
- Safe distance maintenance (0.6m from obstacles)
- Direction selection based on available space

### 2. SLAM Configuration
File: src/auto_robot/config/mapper_params_online_async.yaml
- Online async mapping mode
- Proper sensor integration
- Map saving and loading

### 3. Navigation Configuration
File: src/auto_robot/config/nav2_params.yaml
- Waypoint following for coverage planning
- Behavior tree for decision making
- Costmap configuration for obstacle avoidance

### 4. Custom Behavior Management
File: src/auto_robot/scripts/robot_behavior_manager.py
- Mapping mode activation
- Cleaning mode with waypoint generation
- Docking behavior
- Mode switching via ROS topics

### 5. RViz Controls
File: src/auto_robot/scripts/rviz_cleaning_controls.py
- Start/Stop cleaning commands
- Mode selection panel
- Real-time status display

## Running the System

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

### Build Instructions
```bash
cd /home/selamicetin/Masaüstü/robot
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_robot
```

### Execution Steps
1. Terminal 1 - Start simulation:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch auto_robot launch_sim.launch.py
   ```

2. Terminal 2 - Start visualization:
   ```bash
   source /opt/ros/humble/setup.bash
   rviz2 -d src/auto_robot/config/main.rviz
   ```

3. Terminal 3 - Start autonomous navigation:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/auto_robot/scripts/simple_auto_nav.py
   ```

## Presentation Recording Steps
Detailed instructions are in the presentation_steps/ directory:
1. 01_setup_environment.txt - Environment setup
2. 02_run_simulation.txt - Running Gazebo simulation
3. 03_run_visualization.txt - RViz visualization
4. 04_run_autonomous_navigation.txt - Smart navigation
5. 05_recording_tips.txt - Video recording guidance
6. 06_clean_shutdown.txt - Proper system shutdown

## Docker Deployment
Build and run the containerized version:
```bash
docker build -t homecleanerbot .
docker run --rm -it homecleanerbot
```

## Key Achievements
✅ 2+1 house layout with realistic room arrangement
✅ SLAM mapping capability
✅ Nav2 navigation with waypoint following
✅ Custom robot behavior management
✅ RViz control interface
✅ Docker containerization
✅ Comprehensive documentation
✅ Intelligent obstacle avoidance algorithm

## For Your Presentation Video
1. Show the 2+1 house model in simulation
2. Demonstrate robot movement with obstacle avoidance
3. Display RViz visualization with sensor data
4. Explain the navigation algorithm approach
5. Highlight Docker deployment capability
6. Walk through the system architecture

The system is complete and ready for your final submission!