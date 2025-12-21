#!/bin/bash

# HomeCleanerBot Setup Script
# This script sets up the environment for running the HomeCleanerBot project

echo "🚀 Setting up HomeCleanerBot environment..."

# Update package list
echo "🔄 Updating package list..."
sudo apt update

# Install ROS2 Humble (if not already installed)
echo "📦 Installing ROS2 Humble and dependencies..."
sudo apt install -y ros-humble-desktop \
                    ros-humble-slam-toolbox \
                    ros-humble-navigation2 \
                    ros-humble-nav2-bringup \
                    ros-humble-gazebo-ros-pkgs \
                    python3-colcon-common-extensions \
                    python3-rosdep

# Source ROS2 setup
echo "🔌 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Install rosdep dependencies
echo "🔧 Installing project dependencies with rosdep..."
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-impl urdfdom_headers"

# Build the project
echo "🔨 Building the project..."
colcon build --packages-select auto_robot

echo "✅ Setup complete!"
echo ""
echo "To run the HomeCleanerBot:"
echo "1. Terminal 1 - Start simulation:"
echo "   source setup_env.sh"
echo "   ros2 launch auto_robot launch_sim.launch.py"
echo ""
echo "2. Terminal 2 - Start visualization:"
echo "   source setup_env.sh"
echo "   rviz2 -d src/auto_robot/config/main.rviz"
echo ""
echo "3. Terminal 3 - Start autonomous navigation:"
echo "   source setup_env.sh"
echo "   python3 src/auto_robot/scripts/simple_auto_nav.py"
echo ""
echo "Enjoy your HomeCleanerBot! 🤖"