#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, SaveMap
import yaml
import math
import time

class HomeCleanerController(Node):
    def __init__(self):
        super().__init__('home_cleaner_controller')
        
        # Action clients
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Service clients
        self.save_map_client = self.create_client(SaveMap, '/map_saver/save_map')
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        
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
        self.map_saved = False
        self.using_saved_map = False
        
        self.get_logger().info('HomeCleaner Controller initialized - Ready for RViz commands')
        
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
        elif new_mode == 'load_map_mode':
            self.start_with_saved_map()
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
        
        # Generate exploration waypoints for mapping
        exploration_waypoints = self.generate_exploration_waypoints()
        
        # Send waypoints to navigation system for exploration
        self.send_waypoints(exploration_waypoints)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Mapping started - exploring environment'
        self.status_publisher.publish(status_msg)
        
    def start_with_saved_map(self):
        """Start using a previously saved map"""
        self.current_mode = 'saved_map'
        self.using_saved_map = True
        self.mapping_completed = True  # Consider mapping as completed since we're using saved map
        self.get_logger().info('Started with saved map mode')
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Loaded saved map - ready for cleaning'
        self.status_publisher.publish(status_msg)
        
    def start_cleaning_mode(self):
        """Start cleaning mode with coverage planning"""
        if not self.mapping_completed and not self.using_saved_map:
            self.get_logger().warn('Cannot start cleaning: Mapping not completed and no saved map available')
            # Try to start mapping first
            self.start_mapping_mode()
            return
            
        self.current_mode = 'cleaning'
        self.cleaning_active = True
        self.get_logger().info('Starting cleaning mode with coverage planning')
        
        # Generate coverage waypoints using proper coverage pattern
        waypoints = self.generate_coverage_waypoints()
        
        # Send waypoints to navigation system
        self.send_waypoints(waypoints)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Cleaning started - coverage pattern initiated'
        self.status_publisher.publish(status_msg)
        
    def stop_cleaning_mode(self):
        """Stop cleaning mode"""
        self.current_mode = 'idle'
        self.cleaning_active = False
        self.get_logger().info('Cleaning stopped')
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Cleaning stopped - returning to dock'
        self.status_publisher.publish(status_msg)
        
        # After stopping cleaning, go to docking
        self.dock_robot()
        
    def dock_robot(self):
        """Send robot to docking station"""
        self.current_mode = 'docking'
        self.get_logger().info('Docking robot')
        
        # Define docking station position (this would be predefined)
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = 'map'
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        # Set to a corner of the house world (adjust coordinates as needed)
        dock_pose.pose.position.x = -4.0  # Adjust based on your world
        dock_pose.pose.position.y = -4.0  # Adjust based on your world
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
        
    def generate_exploration_waypoints(self):
        """Generate waypoints for systematic environment exploration during mapping"""
        waypoints = []
        
        # Define exploration area boundaries
        x_start, x_end = -4.0, 4.0
        y_start, y_end = -4.0, 4.0
        step_size = 1.5  # Larger steps for exploration vs cleaning
        
        # Generate a spiral pattern for exploration
        x, y = 0.0, 0.0
        radius = step_size
        angle = 0.0
        
        while radius < 5.0:  # Cover a reasonable exploration area
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            # Point in direction of movement
            waypoint.pose.orientation.w = 1.0
            waypoints.append(waypoint)
            
            angle += 0.5
            radius += 0.1  # Gradually increase radius
            
            if len(waypoints) > 50:  # Limit exploration waypoints
                break
        
        self.get_logger().info(f'Generated {len(waypoints)} exploration waypoints for mapping')
        return waypoints
        
    def generate_coverage_waypoints(self):
        """Generate waypoints for systematic coverage cleaning pattern using boustrophedon pattern"""
        # This is a proper coverage pattern for cleaning
        # In a real implementation, this would use opennav_coverage or similar
        waypoints = []
        
        # Define cleaning area boundaries (based on mapped area)
        # For demonstration, we'll use a grid pattern that covers the house
        x_start, x_end = -3.5, 3.5
        y_start, y_end = -3.5, 3.5
        step_size = 0.7  # Smaller steps for thorough cleaning
        
        # Generate boustrophedon (back-and-forth) pattern
        y = y_start
        direction = 1  # 1 for right, -1 for left
        
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
                    # Orient robot to face direction of movement
                    waypoint.pose.orientation.z = 0.0
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
                    # Orient robot to face direction of movement
                    waypoint.pose.orientation.z = 0.707  # Facing left (180 degrees)
                    waypoint.pose.orientation.w = 0.707
                    waypoints.append(waypoint)
                    x -= step_size
                    
            y += step_size
            direction *= -1  # Reverse direction for next row
            
        self.get_logger().info(f'Generated {len(waypoints)} waypoints for coverage cleaning')
        return waypoints
        
    def send_waypoints(self, waypoints):
        """Send waypoints to the navigation system"""
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return
            
        # Create goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [wp.pose for wp in waypoints]  # Extract Pose from PoseStamped
        
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
        elif self.current_mode == 'mapping':
            # After mapping is done, save the map
            self.save_current_map()
            self.mapping_completed = True
            self.map_saved = True
            status_msg = String()
            status_msg.data = 'Mapping completed and saved'
            self.status_publisher.publish(status_msg)
        
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
    
    def save_current_map(self):
        """Save the current map to disk"""
        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map saver service not available')
            return False
            
        # Create save map request
        request = SaveMap.Request()
        request.map_url = "/home/selamicetin/Masaüstü/robot3/maps/home_cleaner_map"
        request.image_format = "png"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        self.get_logger().info('Saving current map...')
        future = self.save_map_client.call_async(request)
        future.add_done_callback(self.save_map_callback)
        return True
    
    def save_map_callback(self, future):
        """Callback for map saving result"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Map saved successfully')
                self.map_saved = True
            else:
                self.get_logger().error(f'Failed to save map: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Exception while saving map: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    controller = HomeCleanerController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()