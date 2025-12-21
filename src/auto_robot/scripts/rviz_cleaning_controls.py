#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CleaningControlPanel(Node):
    def __init__(self):
        super().__init__('cleaning_control_panel')
        
        # Publisher for robot mode commands
        self.mode_publisher = self.create_publisher(String, 'robot_mode', 10)
        
        self.get_logger().info('Cleaning Control Panel initialized')
        self.get_logger().info('Use publish_start_cleaning() and publish_stop_cleaning() methods to control the robot')
        
    def publish_start_cleaning(self):
        """Publish start cleaning command"""
        msg = String()
        msg.data = 'start_cleaning'
        self.mode_publisher.publish(msg)
        self.get_logger().info('Published start cleaning command')
        
    def publish_stop_cleaning(self):
        """Publish stop cleaning command"""
        msg = String()
        msg.data = 'stop_cleaning'
        self.mode_publisher.publish(msg)
        self.get_logger().info('Published stop cleaning command')
        
    def publish_start_mapping(self):
        """Publish start mapping command"""
        msg = String()
        msg.data = 'start_mapping'
        self.mode_publisher.publish(msg)
        self.get_logger().info('Published start mapping command')
        
    def publish_dock_robot(self):
        """Publish dock robot command"""
        msg = String()
        msg.data = 'dock'
        self.mode_publisher.publish(msg)
        self.get_logger().info('Published dock robot command')

def main(args=None):
    rclpy.init(args=args)
    
    control_panel = CleaningControlPanel()
    
    try:
        rclpy.spin(control_panel)
    except KeyboardInterrupt:
        pass
    finally:
        control_panel.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()