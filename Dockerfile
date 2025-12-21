# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install prerequisites
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace directory
WORKDIR /home/ros2_ws

# Copy source code
COPY . /home/ros2_ws/src/homecleanerbot

# Install dependencies with rosdep
RUN rosdep init || true
RUN rosdep update
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the workspace
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entry point
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && exec \"$@\"", "/bin/bash"]