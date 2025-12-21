# HomeCleanerBot

An autonomous indoor cleaning robot simulation using ROS2, Gazebo, and Navigation2.

## Project Overview

This project implements a complete autonomous robot system for indoor cleaning with obstacle avoidance capabilities. The robot operates in a custom-designed 2+1 house layout (two bedrooms and a living room) and demonstrates intelligent navigation behaviors.



## Features

- **Enhanced 2+1 House Layout**: Custom SDF world with realistic room arrangement, furniture, and decorative elements
- **Intelligent Navigation**: Obstacle avoidance without collisions
- **SLAM Mapping**: Simultaneous Localization and Mapping capabilities
- **Path Planning**: Coverage planning using Navigation2 waypoint following
- **Sensor Integration**: LiDAR, camera, and depth camera sensors
- **Docker Support**: Containerized deployment for easy setup
- **Improved Robot Design**: Modern aesthetics with layered visual elements

## System Architecture

```
├── House Environment (Gazebo SDF)
├── Robot Model (URDF/XACRO)
├── SLAM Toolbox (Mapping)
├── Navigation2 (Path Planning)
├── Custom Control Scripts (Python)
└── Visualization (RViz)
```

## Prerequisites

### Operating System
- Ubuntu 22.04 LTS (Jammy Jellyfish)

### Software Versions
- **ROS2**: Humble Hawksbill
- **Gazebo**: Fortress 11.10+
- **Python**: 3.10+
- **Navigation2**: 1.1.5+
- **SLAM Toolbox**: 2.6.4+

## Installation

```bash
# Clone the repository
git clone https://github.com/Selami7321/robotic-project.git
cd robotic-project

# Run setup script
./setup_project.sh
```

## Usage

1. **Start Simulation**:
   ```bash
   source setup_env.sh
   ros2 launch auto_robot launch_sim.launch.py
   ```

2. **Start Visualization**:
   ```bash
   source setup_env.sh
   rviz2 -d src/auto_robot/config/main.rviz
   ```

3. **Start Autonomous Navigation**:
   ```bash
   source setup_env.sh
   python3 src/auto_robot/scripts/simple_auto_nav.py
   ```

## Project Structure

```
src/auto_robot/
├── config/          # SLAM and Navigation parameters
├── description/     # Robot URDF/XACRO models
├── launch/          # ROS2 launch files
├── scripts/         # Custom Python scripts
├── worlds/          # Gazebo world files
└── package.xml      # Package definition
```

## Key Components

### Robot Design
- Differential drive base
- RGB camera
- Depth camera
- 360° LiDAR sensor
- Enhanced visual aesthetics with layered design
- Modern color scheme (red chassis with white accents)
- Detailed wheel design with black tire edges
- Improved caster wheel with silver detailing

### Navigation Algorithm
The `simple_auto_nav.py` script implements an intelligent obstacle avoidance system:
- Sector-based LiDAR analysis
- State machine approach (FORWARD, TURN_LEFT, TURN_RIGHT)
- Safe distance maintenance (0.6m)
- Direction selection based on available space

### World Model
The `house.world` file defines an enhanced 2+1 layout with:
- Two bedrooms with beds and nightstands
- Living room with sofa, coffee table, TV stand, and rug
- Corridors with doors
- Decorative elements (plants, rugs)
- Realistic furniture placement
- Improved visual aesthetics

## Docker Deployment

```bash
# Build the image
docker build -t homecleanerbot .

# Run the container
docker run --rm -it homecleanerbot
```

## Presentation Materials

Detailed instructions for demonstrating the system are available in the `presentation_steps/` directory.

## Author

Selami Cetin - [Selami7321](https://github.com/Selami7321)

## License

This project is licensed under the MIT License - see the LICENSE file for details.