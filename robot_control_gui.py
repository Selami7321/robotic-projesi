#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import subprocess
import threading

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("HomeCleanerBot Control Panel")
        self.root.geometry("400x300")
        
        # Create main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="HomeCleanerBot Control Panel", font=("Arial", 14, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Mapping buttons
        ttk.Label(main_frame, text="Mapping:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky=tk.W, pady=5)
        ttk.Button(main_frame, text="Start Mapping", command=self.start_mapping).grid(row=1, column=1, padx=5, pady=5)
        
        # Cleaning buttons
        ttk.Label(main_frame, text="Cleaning:", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky=tk.W, pady=5)
        ttk.Button(main_frame, text="Start Cleaning", command=self.start_cleaning).grid(row=2, column=1, padx=5, pady=5)
        ttk.Button(main_frame, text="Stop Cleaning", command=self.stop_cleaning).grid(row=3, column=1, padx=5, pady=5)
        
        # Docking button
        ttk.Label(main_frame, text="Docking:", font=("Arial", 10, "bold")).grid(row=4, column=0, sticky=tk.W, pady=5)
        ttk.Button(main_frame, text="Dock Robot", command=self.dock_robot).grid(row=4, column=1, padx=5, pady=5)
        
        # Obstacle avoidance
        ttk.Label(main_frame, text="Navigation:", font=("Arial", 10, "bold")).grid(row=5, column=0, sticky=tk.W, pady=5)
        ttk.Button(main_frame, text="Obstacle Avoidance", command=self.obstacle_avoidance).grid(row=5, column=1, padx=5, pady=5)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Ready", foreground="green")
        self.status_label.grid(row=6, column=0, columnspan=2, pady=20)
        
        # Emergency stop
        stop_button = ttk.Button(main_frame, text="EMERGENCY STOP", command=self.emergency_stop)
        stop_button.grid(row=7, column=0, columnspan=2, pady=10)
        stop_button.configure(style='Emergency.TButton')
        
        # Configure button style for emergency stop
        style = ttk.Style()
        style.configure('Emergency.TButton', foreground='white', background='red')
        
    def send_command(self, command):
        """Send a command to the robot in a separate thread"""
        def run_command():
            try:
                self.status_label.config(text=f"Sending: {command}", foreground="orange")
                cmd = f"ros2 topic pub /robot_mode std_msgs/String \"data: '{command}'\""
                result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    self.status_label.config(text=f"Command '{command}' sent successfully", foreground="green")
                else:
                    self.status_label.config(text=f"Error: {result.stderr}", foreground="red")
            except subprocess.TimeoutExpired:
                self.status_label.config(text="Command timeout", foreground="red")
            except Exception as e:
                self.status_label.config(text=f"Error: {str(e)}", foreground="red")
        
        # Run command in separate thread to avoid blocking GUI
        thread = threading.Thread(target=run_command)
        thread.daemon = True
        thread.start()
    
    def start_mapping(self):
        self.send_command("start_mapping")
    
    def start_cleaning(self):
        self.send_command("start_cleaning")
    
    def stop_cleaning(self):
        self.send_command("stop_cleaning")
    
    def dock_robot(self):
        self.send_command("dock")
    
    def obstacle_avoidance(self):
        self.send_command("obstacle_avoidance")
    
    def emergency_stop(self):
        # Send a stop command to cmd_vel directly
        def stop_motion():
            try:
                cmd = "ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once"
                subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=2)
                self.status_label.config(text="Emergency stop executed", foreground="red")
            except:
                self.status_label.config(text="Stop command failed", foreground="red")
        
        thread = threading.Thread(target=stop_motion)
        thread.daemon = True
        thread.start()

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()