import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('auto_robot')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'house.world'),
        description='Full path to world model file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Include the robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        )
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'home_cleaner_bot'],
        output='screen')

    # Start Map Server with pre-saved map
    start_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'nav2_params.yaml')]
    )

    # Start Nav2 (with localization instead of SLAM)
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # Start AMCL for localization
    start_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'map': '/home/selamicetin/Masaüstü/robot3/maps/home_cleaner_map.yaml'  # Path to saved map
        }.items()
    )

    # Start Lifecycle Manager for Nav2
    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'planner_server', 'controller_server', 
                                   'recoveries_server', 'bt_navigator', 'waypoint_follower',
                                   'coverage_server']}]
    )

    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'config', 'main.rviz')],
        output='screen'
    )

    # Start our custom behavior manager
    start_behavior_manager = Node(
        package='auto_robot',
        executable='robot_behavior_manager.py',
        name='home_cleaner_bot',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(start_map_server)
    ld.add_action(start_amcl)  # Use AMCL for localization with saved map
    ld.add_action(start_lifecycle_manager)
    ld.add_action(start_nav2)
    ld.add_action(start_rviz)
    ld.add_action(start_behavior_manager)

    return ld