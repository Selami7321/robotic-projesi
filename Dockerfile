# Use ROS2 Humble with Ubuntu 22.04
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV GAZEBO_VERSION=garden

# Set working directory
WORKDIR /home_cleaner_bot_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    git \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 navigation and SLAM packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install coverage planning dependencies (opennav_coverage)
RUN apt-get update && apt-get install -y \
    ros-humble-opennav-coverage \
    ros-humble-opennav-coverage-bt \
    && rm -rf /var/lib/apt/lists/*

# Copy the project files to the workspace
COPY . /home_cleaner_bot_ws/

# Set proper permissions
RUN chmod +x /home_cleaner_bot_ws/src/auto_robot/scripts/*.py

# Source ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home_cleaner_bot_ws/install/setup.bash" >> /root/.bashrc

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home_cleaner_bot_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-select auto_robot"

# Set the default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /home_cleaner_bot_ws/install/setup.bash && ros2 launch auto_robot home_cleaner.launch.py"]