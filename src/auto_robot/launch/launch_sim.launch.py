import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='auto_robot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'house.world')}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'auto_car',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.05',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', '0.0'],
                        output='screen')


    # Start SLAM Toolbox for mapping
    start_slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')],
    )
    
    # Start Nav2 navigation stack
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true'
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
    
    # Start the robot behavior manager
    start_robot_behavior_manager = Node(
        package='auto_robot',
        executable='robot_behavior_manager.py',
        name='robot_behavior_manager',
        output='screen',
    )
    
    # Start the coverage planner
    start_coverage_planner = Node(
        package='auto_robot',
        executable='coverage_planner.py',
        name='coverage_planner',
        output='screen',
    )
    
    # Start the custom navigation script
    start_simple_nav = Node(
        package='auto_robot',
        executable='simple_auto_nav.py',
        name='simple_auto_nav',
        output='screen',
    )
    
    # Start the battery simulator
    start_battery_sim = Node(
        package='auto_robot',
        executable='battery_simulator.py',
        name='battery_simulator',
        output='screen',
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        start_slam_toolbox,  # Add SLAM for mapping capability
        start_lifecycle_manager,  # Add lifecycle manager
        start_nav2,  # Add full Nav2 stack for navigation
        start_robot_behavior_manager,  # Add robot behavior manager
        start_coverage_planner,  # Add coverage planner
        start_simple_nav,  # Add simple navigation
        start_battery_sim,  # Add battery simulator
    ])