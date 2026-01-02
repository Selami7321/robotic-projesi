#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        
        # Publisher for battery level
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        
        # Subscriber for robot position to calculate movement
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Timer for battery updates
        self.battery_timer = self.create_timer(1.0, self.update_battery)  # Update every second
        
        # Battery parameters
        self.battery_level = 100.0  # Start fully charged
        self.last_position = None
        self.battery_drain_rate = 0.05  # Drain 0.05% per second when moving
        self.movement_threshold = 0.01  # Consider movement above this threshold
        
        self.get_logger().info('Battery Simulator initialized')
        
    def odom_callback(self, msg):
        """Track robot movement to calculate battery drain"""
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.last_position is not None:
            # Calculate distance moved
            distance = math.sqrt(
                (current_position[0] - self.last_position[0])**2 + 
                (current_position[1] - self.last_position[1])**2
            )
            
            # If moving significantly, drain battery faster
            if distance > self.movement_threshold:
                # Additional drain when moving
                self.battery_level -= self.battery_drain_rate * 3  # 3x drain when moving
                self.get_logger().info(f'Battery drained due to movement: {self.battery_level:.2f}%')
        
        self.last_position = current_position
        
    def update_battery(self):
        """Update battery level periodically"""
        # Natural battery drain over time
        self.battery_level -= self.battery_drain_rate
        
        if self.battery_level < 0:
            self.battery_level = 0
            
        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)
        
        # Log battery level periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f'Battery Level: {self.battery_level:.2f}%')
            
        # If battery is low, publish warning
        if self.battery_level < 20.0:
            self.get_logger().warn(f'LOW BATTERY: {self.battery_level:.2f}% - Initiating docking sequence!')

def main(args=None):
    rclpy.init(args=args)
    
    battery_simulator = BatterySimulator()
    
    try:
        rclpy.spin(battery_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        battery_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()