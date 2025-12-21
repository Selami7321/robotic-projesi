# HomeCleanerBot Docker Instructions

## Building the Docker Image

To build the Docker image, run the following command from the project root directory:

```bash
docker build -t homecleanerbot .
```

## Running the Docker Container

### Using Docker Run

To run the container directly with Docker:

```bash
docker run --rm -it \
  --privileged \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/maps:/home/ros2_ws/maps \
  homecleanerbot \
  bash -c "source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && ros2 launch auto_robot home_cleaner.launch.py"
```

### Using Docker Compose

Alternatively, you can use docker-compose:

```bash
docker-compose up
```

## Stopping the Container

To stop the container when using docker-compose:

```bash
docker-compose down
```

When using docker run, simply press Ctrl+C in the terminal where the container is running.

## Accessing the Container

To access a running container's shell:

```bash
docker exec -it homecleanerbot bash
```

## Image Details

- Base OS: Ubuntu 22.04
- ROS2 Distribution: Humble
- Included Packages:
  - ROS2 Humble Desktop
  - SLAM Toolbox
  - Navigation2
  - Gazebo ROS Packages
  - Colcon build tools