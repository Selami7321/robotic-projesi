#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
import yaml
import math

class HomeCleanerBot(Node):
    def __init__(self):
        super().__init__('home_cleaner_bot')
        
        # Action clients
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Service clients
        self.start_slam_client = self.create_client(Empty, 'start_slam_mapping')
        self.save_map_client = self.create_client(Empty, 'save_map')
        
        # Subscribers
        self.mode_subscription = self.create_subscription(
            String,
            'robot_mode',
            self.mode_callback,
            10)
            
        # Publishers
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # State variables
        self.current_mode = 'idle'
        self.mapping_completed = False
        self.cleaning_active = False
        self.docked = False
        
        self.get_logger().info('HomeCleanerBot initialized')
        
    def mode_callback(self, msg):
        """Callback for mode change commands"""
        new_mode = msg.data.lower()
        self.get_logger().info(f'Received mode command: {new_mode}')
        
        if new_mode == 'start_mapping':
            self.start_mapping_mode()
        elif new_mode == 'start_cleaning':
            self.start_cleaning_mode()
        elif new_mode == 'stop_cleaning':
            self.stop_cleaning_mode()
        elif new_mode == 'dock':
            self.dock_robot()
        else:
            self.get_logger().warn(f'Unknown mode command: {new_mode}')
            
    def timer_callback(self):
        """Periodic callback for status updates"""
        status_msg = String()
        status_msg.data = f'Current mode: {self.current_mode}'
        self.status_publisher.publish(status_msg)
        
    def start_mapping_mode(self):
        """Start SLAM mapping mode"""
        self.current_mode = 'mapping'
        self.get_logger().info('Starting mapping mode')
        
        # In a real implementation, this would start SLAM mapping
        # For now, we'll simulate the process
        self.mapping_completed = False
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Mapping started'
        self.status_publisher.publish(status_msg)
        
    def start_cleaning_mode(self):
        """Start cleaning mode with coverage planning"""
        if not self.mapping_completed:
            self.get_logger().warn('Cannot start cleaning: Mapping not completed')
            return
            
        self.current_mode = 'cleaning'
        self.cleaning_active = True
        self.get_logger().info('Starting cleaning mode')
        
        # Generate coverage waypoints
        waypoints = self.generate_coverage_waypoints()
        
        # Send waypoints to navigation system
        self.send_waypoints(waypoints)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Cleaning started'
        self.status_publisher.publish(status_msg)
        
    def stop_cleaning_mode(self):
        """Stop cleaning mode"""
        self.current_mode = 'idle'
        self.cleaning_active = False
        self.get_logger().info('Cleaning stopped')
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Cleaning stopped'
        self.status_publisher.publish(status_msg)
        
    def dock_robot(self):
        """Send robot to docking station"""
        self.current_mode = 'docking'
        self.get_logger().info('Docking robot')
        
        # Define docking station position (this would be predefined)
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = 'map'
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        dock_pose.pose.position.x = 0.0
        dock_pose.pose.position.y = 0.0
        dock_pose.pose.orientation.w = 1.0
        
        # Send robot to docking station
        self.navigate_to_pose(dock_pose)
        
        # Update state
        self.docked = True
        self.cleaning_active = False
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Docking initiated'
        self.status_publisher.publish(status_msg)
        
    def generate_coverage_waypoints(self):
        """Generate waypoints for coverage cleaning pattern"""
        # This is a simplified spiral coverage pattern
        # In a real implementation, this would use a more sophisticated algorithm
        waypoints = []
        
        # Define cleaning area boundaries (based on mapped area)
        # For demonstration, we'll use a simple grid pattern
        x_start, x_end = -4.0, 4.0
        y_start, y_end = -4.0, 4.0
        step_size = 1.0
        
        # Generate a simple back-and-forth pattern
        y = y_start
        direction = 1
        
        while y <= y_end:
            if direction == 1:
                # Move left to right
                x = x_start
                while x <= x_end:
                    waypoint = PoseStamped()
                    waypoint.header.frame_id = 'map'
                    waypoint.header.stamp = self.get_clock().now().to_msg()
                    waypoint.pose.position.x = x
                    waypoint.pose.position.y = y
                    waypoint.pose.orientation.w = 1.0
                    waypoints.append(waypoint)
                    x += step_size
            else:
                # Move right to left
                x = x_end
                while x >= x_start:
                    waypoint = PoseStamped()
                    waypoint.header.frame_id = 'map'
                    waypoint.header.stamp = self.get_clock().now().to_msg()
                    waypoint.pose.position.x = x
                    waypoint.pose.position.y = y
                    waypoint.pose.orientation.w = 1.0
                    waypoints.append(waypoint)
                    x -= step_size
                    
            y += step_size
            direction *= -1  # Reverse direction
            
        self.get_logger().info(f'Generated {len(waypoints)} waypoints for coverage cleaning')
        return waypoints
        
    def send_waypoints(self, waypoints):
        """Send waypoints to the navigation system"""
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return
            
        # Create goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        # Send goal
        future = self.follow_waypoints_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_response_callback)
        
    def navigate_to_pose(self, pose):
        """Navigate to a specific pose"""
        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return
            
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal
        future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_response_callback)
        
    def waypoint_response_callback(self, future):
        """Callback for waypoint following response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint following goal rejected')
            return
            
        self.get_logger().info('Waypoint following goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result_callback)
        
    def waypoint_result_callback(self, future):
        """Callback for waypoint following result"""
        result = future.result().result
        self.get_logger().info('Waypoint following completed')
        
        # After cleaning is done, go to docking
        if self.current_mode == 'cleaning':
            self.dock_robot()
        
    def navigation_response_callback(self, future):
        """Callback for navigation response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return
            
        self.get_logger().info('Navigation goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
        
    def navigation_result_callback(self, future):
        """Callback for navigation result"""
        result = future.result().result
        self.get_logger().info('Navigation completed')
        
        # If we were docking, update state
        if self.current_mode == 'docking':
            self.current_mode = 'idle'
            self.docked = True
            status_msg = String()
            status_msg.data = 'Docking completed'
            self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    robot = HomeCleanerBot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()