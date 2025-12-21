#!/bin/bash

# HomeCleanerBot Environment Setup Script
# This script sets up the environment variables for running the HomeCleanerBot

echo "🔧 Setting up HomeCleanerBot environment..."

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source the project workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Project workspace sourced successfully"
else
    echo "⚠️  Warning: Project workspace not found. Please build the project first with 'colcon build'"
fi

echo "🤖 HomeCleanerBot environment ready!"
echo ""
echo "Available commands:"
echo "  ros2 launch auto_robot launch_sim.launch.py     # Start simulation"
echo "  ros2 launch auto_robot simple_gazebo.launch.py  # Start simple gazebo"
echo "  rviz2 -d src/auto_robot/config/main.rviz       # Start visualization"
echo "  python3 src/auto_robot/scripts/simple_auto_nav.py  # Start navigation"