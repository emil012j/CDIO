#!/usr/bin/env python3

import time
import threading
import sys
import os

# Add parent directory to path so we can import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.navigation_controller import NavigationController
from robot.connection.network import NetworkClient

class RobotController:
    """
    Main robot controller class that runs on the EV3 robot.
    
    This controller:
    1. Establishes a connection with the PC running the camera
    2. Receives coordinates and object data
    3. Manages navigation to targets
    """
    
    def __init__(self, server_ip='192.168.0.100', server_port=5000):
        """Initialize the robot controller with connection details"""
        self.server_ip = server_ip
        self.server_port = server_port
        self.running = False
        
        # Initialize network client
        self.network = NetworkClient(server_ip, server_port)
        
        # Navigation controller
        self.nav_controller = NavigationController()
        
        # Status tracking
        self.last_update_time = time.time()
        self.last_action = "NONE"
        
        print("Robot controller initialized. Will connect to {0}:{1}".format(server_ip, server_port))
    
    def data_received_callback(self, data):
        """Callback function for processing received data"""
        # Update the navigation controller with received data
        if self.nav_controller.update_from_camera_data(data):
            self.last_update_time = time.time()
    
    def run(self):
        """Main robot control loop"""
        if not self.nav_controller.initialized:
            print("Navigation controller not initialized. Cannot run.")
            return
        
        # Connect to server
        if not self.network.connect():
            print("Failed to connect to server. Exiting.")
            return
        
        # Set callback and start receiver
        self.network.set_data_callback(self.data_received_callback)
        if not self.network.start_receiver():
            print("Failed to start data receiver. Exiting.")
            self.network.disconnect()
            return
        
        # Main control loop
        try:
            print("Starting main control loop...")
            self.running = True
            
            # Wait a bit for initial data
            time.sleep(1)
            
            while self.running:
                # Check if we've received data recently
                current_time = time.time()
                if current_time - self.last_update_time > 3.0:
                    # If no data received for a while, stop moving
                    self.nav_controller.stop()
                    if self.last_action != "STOPPED_NO_DATA":
                        print("Stopped: No valid data received for 3 seconds")
                        self.last_action = "STOPPED_NO_DATA"
                else:
                    # Execute navigation
                    action = self.nav_controller.navigate()
                    
                    # Only log when action changes
                    if action != self.last_action:
                        print("Navigation action: {0}".format(action))
                        self.last_action = action
                
                # Sleep a short time to avoid consuming too much CPU
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            # Clean up
            self.running = False
            self.nav_controller.stop()
            self.network.disconnect()
            print("Robot controller stopped")
    
    def stop(self):
        """Stop the robot controller and all movements"""
        self.running = False
        
        if self.nav_controller:
            self.nav_controller.stop()
            
        if self.network:
            self.network.disconnect()

if __name__ == "__main__":
    # Parse command line arguments for server IP and port
    server_ip = '192.168.0.100'  # Default IP
    server_port = 5000           # Default port
    
    if len(sys.argv) >= 2:
        server_ip = sys.argv[1]
        
    if len(sys.argv) >= 3:
        try:
            server_port = int(sys.argv[2])
        except ValueError:
            print("Invalid port number: {0}, using default: {1}".format(sys.argv[2], server_port))
    
    # Create and run the robot controller
    controller = RobotController(server_ip, server_port)
    controller.run() 