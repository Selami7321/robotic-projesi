#!/bin/bash

# Simple script to send robot commands

case "$1" in
    "start_mapping")
        echo "Sending start mapping command..."
        ros2 topic pub /robot_mode std_msgs/String "data: 'start_mapping'"
        ;;
    "start_cleaning")
        echo "Sending start cleaning command..."
        ros2 topic pub /robot_mode std_msgs/String "data: 'start_cleaning'"
        ;;
    "stop_cleaning")
        echo "Sending stop cleaning command..."
        ros2 topic pub /robot_mode std_msgs/String "data: 'stop_cleaning'"
        ;;
    "dock")
        echo "Sending dock command..."
        ros2 topic pub /robot_mode std_msgs/String "data: 'dock'"
        ;;
    "obstacle_avoidance")
        echo "Sending obstacle avoidance command..."
        ros2 topic pub /robot_mode std_msgs/String "data: 'obstacle_avoidance'"
        ;;
    *)
        echo "Usage: $0 {start_mapping|start_cleaning|stop_cleaning|dock|obstacle_avoidance}"
        echo "Available commands:"
        echo "  start_mapping      - Start mapping mode"
        echo "  start_cleaning     - Start cleaning mode"
        echo "  stop_cleaning      - Stop cleaning and dock"
        echo "  dock              - Dock the robot"
        echo "  obstacle_avoidance - Return to obstacle avoidance mode"
        ;;
esac