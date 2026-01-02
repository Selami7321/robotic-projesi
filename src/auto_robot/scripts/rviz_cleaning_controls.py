#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class CleaningControlPanel(Node):
    def __init__(self):
        super().__init__('cleaning_control_panel')
        
        # Publisher for robot mode commands
        self.mode_publisher = self.create_publisher(String, 'robot_mode', 10)
        
        self.get_logger().info('Cleaning Control Panel initialized')
        self.get_logger().info('Use command line arguments to control the robot: start_cleaning, stop_cleaning, start_mapping, dock')
        
    def publish_command(self, command):
        """Publish a command to the robot"""
        msg = String()
        msg.data = command
        self.mode_publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')

def main(args=None):
    rclpy.init(args=args)
    
    control_panel = CleaningControlPanel()
    
    # Process command line arguments
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command in ['start_cleaning', 'stop_cleaning', 'start_mapping', 'dock', 'load_map_mode']:
            control_panel.publish_command(command)
        else:
            control_panel.get_logger().warn(f'Unknown command: {command}')
            control_panel.get_logger().info('Available commands: start_cleaning, stop_cleaning, start_mapping, dock, load_map_mode')
    else:
        control_panel.get_logger().info('Usage: ros2 run auto_robot rviz_cleaning_controls.py <command>')
        control_panel.get_logger().info('Available commands: start_cleaning, stop_cleaning, start_mapping, dock, load_map_mode')
    
    # Keep the node alive briefly to allow message to be sent
    control_panel.get_logger().info('Command sent. Node will shut down in 1 second.')
    timer = control_panel.create_timer(1.0, lambda: rclpy.shutdown())
    
    try:
        rclpy.spin(control_panel)
    except KeyboardInterrupt:
        pass
    finally:
        control_panel.destroy_node()

if __name__ == '__main__':
    main()