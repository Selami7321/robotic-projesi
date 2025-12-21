#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SmartAutoNavigator(Node):
    def __init__(self):
        super().__init__('smart_auto_navigator')
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Store latest scan data
        self.latest_scan = None
        
        # Robot state
        self.linear_vel = 0.2
        self.angular_vel = 0.0
        self.state = 'FORWARD'  # FORWARD, TURN_LEFT, TURN_RIGHT
        self.turn_start_time = None
        
        self.get_logger().info('Smart Auto Navigator started')
        self.get_logger().info('Robot will intelligently navigate and avoid obstacles')

    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.latest_scan = msg

    def get_sector_distance(self, msg, sector_center_deg, sector_width_deg):
        """Get minimum distance in a sector of the laser scan"""
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        
        # Convert to indices
        center_index = int((sector_center_deg * math.pi / 180.0) / angle_increment + num_readings / 2)
        half_width_indices = int((sector_width_deg * math.pi / 180.0) / (2 * angle_increment))
        
        min_distance = float('inf')
        start_idx = max(0, center_index - half_width_indices)
        end_idx = min(num_readings, center_index + half_width_indices)
        
        for i in range(start_idx, end_idx):
            if not math.isnan(msg.ranges[i]) and msg.ranges[i] > 0.1:  # Filter out invalid readings
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i]
        
        return min_distance if min_distance != float('inf') else 10.0  # Return large value if no obstacles

    def timer_callback(self):
        """Timer callback to send velocity commands"""
        if self.latest_scan is None:
            return
            
        cmd = Twist()
        
        # Get distances in different sectors
        front_distance = self.get_sector_distance(self.latest_scan, 0, 30)    # Front 30 degrees
        left_distance = self.get_sector_distance(self.latest_scan, 90, 30)   # Left 30 degrees
        right_distance = self.get_sector_distance(self.latest_scan, -90, 30) # Right 30 degrees
        
        safe_distance = 0.6  # Minimum safe distance from obstacles
        
        # State machine for navigation
        if self.state == 'FORWARD':
            if front_distance < safe_distance:
                # Obstacle ahead, decide which direction to turn
                if left_distance > right_distance:
                    self.state = 'TURN_LEFT'
                    self.turn_start_time = self.get_clock().now()
                    self.get_logger().info('Obstacle ahead! Turning left...')
                else:
                    self.state = 'TURN_RIGHT'
                    self.turn_start_time = self.get_clock().now()
                    self.get_logger().info('Obstacle ahead! Turning right...')
            else:
                # Move forward
                cmd.linear.x = self.linear_vel
                cmd.angular.z = 0.0
                
        elif self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn left
            
            # Turn for 1.5 seconds
            if self.turn_start_time is not None:
                elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
                if elapsed > 1.5:
                    self.state = 'FORWARD'
                    self.get_logger().info('Turn completed. Moving forward.')
                    
        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5  # Turn right
            
            # Turn for 1.5 seconds
            if self.turn_start_time is not None:
                elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
                if elapsed > 1.5:
                    self.state = 'FORWARD'
                    self.get_logger().info('Turn completed. Moving forward.')
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    navigator = SmartAutoNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        navigator.cmd_vel_pub.publish(cmd)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()