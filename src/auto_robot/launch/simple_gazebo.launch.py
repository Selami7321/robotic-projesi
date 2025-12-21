import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='auto_robot'

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file with empty world
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')}.items()
             )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'auto_car'],
                        output='screen')

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])