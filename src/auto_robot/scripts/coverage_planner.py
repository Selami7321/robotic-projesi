#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        
        # Action client for computing paths
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # Publisher for coverage path visualization
        self.coverage_path_pub = self.create_publisher(Path, 'coverage_path', 10)
        
        # Publisher for robot commands
        self.coverage_cmd_pub = self.create_publisher(String, 'coverage_command', 10)
        
        # Coverage parameters
        self.robot_width = 0.5  # Estimated robot width
        self.coverage_spacing = 0.4  # Distance between coverage lines
        self.room_boundaries = None  # Will be set based on the house layout
        
        self.get_logger().info('Coverage Planner initialized')
        
    def compute_coverage_path(self, start_pose, room_min_x, room_max_x, room_min_y, room_max_y):
        """Compute a coverage path using boustrophedon (lawnmower) pattern"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Implement boustrophedon pattern for coverage
        y = room_min_y + self.coverage_spacing
        direction = 1  # 1 for left-to-right, -1 for right-to-left
        
        while y < room_max_y:
            if direction == 1:
                # Left to right pass
                start_point = PoseStamped()
                start_point.header.frame_id = "map"
                start_point.pose.position.x = room_min_x + 0.2  # Add margin
                start_point.pose.position.y = y
                start_point.pose.orientation.w = 1.0
                path.poses.append(start_point)
                
                end_point = PoseStamped()
                end_point.header.frame_id = "map"
                end_point.pose.position.x = room_max_x - 0.2  # Add margin
                end_point.pose.position.y = y
                end_point.pose.orientation.w = 1.0
                path.poses.append(end_point)
            else:
                # Right to left pass
                start_point = PoseStamped()
                start_point.header.frame_id = "map"
                start_point.pose.position.x = room_max_x - 0.2  # Add margin
                start_point.pose.position.y = y
                start_point.pose.orientation.w = 1.0
                path.poses.append(start_point)
                
                end_point = PoseStamped()
                end_point.header.frame_id = "map"
                end_point.pose.position.x = room_min_x + 0.2  # Add margin
                end_point.pose.position.y = y
                end_point.pose.orientation.w = 1.0
                path.poses.append(end_point)
            
            y += self.coverage_spacing
            direction *= -1  # Reverse direction for next pass
        
        # Publish the coverage path for visualization
        self.coverage_path_pub.publish(path)
        self.get_logger().info(f'Coverage path computed with {len(path.poses)} waypoints')
        
        return path

    def execute_coverage_mission(self):
        """Execute the full coverage mission for the house"""
        self.get_logger().info('Starting coverage mission...')
        
        # Define room boundaries based on house.world layout
        # Room 1: Living room area
        self.execute_room_coverage(-0.1, 5.8, -5.8, -0.1, "living_room")
        
        # Room 2: Dining/kitchen area  
        self.execute_room_coverage(-0.1, 5.8, 0.1, 5.8, "dining_kitchen")
        
        # Room 3: Bedroom 1
        self.execute_room_coverage(-5.8, -0.1, 0.1, 5.8, "bedroom_1")
        
        # Room 4: Bedroom 2
        self.execute_room_coverage(-5.8, -0.1, -5.8, -0.1, "bedroom_2")
        
        self.get_logger().info('Coverage mission completed!')

    def execute_room_coverage(self, min_x, max_x, min_y, max_y, room_name):
        """Execute coverage for a specific room"""
        self.get_logger().info(f'Covering {room_name}: x[{min_x}, {max_x}], y[{min_y}, {max_y}]')
        
        # Compute coverage path for this room
        dummy_pose = PoseStamped()
        dummy_pose.header.frame_id = "map"
        coverage_path = self.compute_coverage_path(dummy_pose, min_x, max_x, min_y, max_y)
        
        # Send path to navigation system for execution
        cmd_msg = String()
        cmd_msg.data = f"start_coverage_{room_name}"
        self.coverage_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    
    coverage_planner = CoveragePlanner()
    
    # Execute the coverage mission
    coverage_planner.execute_coverage_mission()
    
    try:
        rclpy.spin(coverage_planner)
    except KeyboardInterrupt:
        pass
    finally:
        coverage_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()