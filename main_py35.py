#!/usr/bin/env python3
"""
Main program for EV3 robot control with camera-based navigation

This program provides a command-line interface to run either:
1. The PC-side camera processing and server
2. The EV3 robot controller (if running on the EV3)

Author: Team CDIO
"""

import os
import sys
import argparse
import time
import socket

def get_local_ip():
    """Get the local IP address of this machine"""
    try:
        # Create a socket to determine the IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Doesn't need to be reachable
        s.connect(('8.8.8.8', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'  # Fallback to localhost

def is_running_on_ev3():
    """Check if the script is running on an EV3 device"""
    try:
        # Check for ev3dev-specific directories
        return os.path.exists('/sys/class/lego-sensor') or os.path.exists('/sys/class/lego-port')
    except:
        return False

def run_camera_server(port=5000, model_path=None):
    """Run the camera server on PC side"""
    try:
        from src.camera.camera import CameraProcessor
        
        ip = get_local_ip()
        print("Starting camera server on {0}:{1}".format(ip, port))
        
        # Determine model path
        if model_path is None:
            model_path = "src/camera/best.pt"
            if not os.path.exists(model_path):
                model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                         "src/camera/best.pt")
        
        # Create and run camera processor
        processor = CameraProcessor(
            model_path=model_path,
            host='0.0.0.0',  # Listen on all interfaces
            port=port
        )
        
        print("Camera server ready! Configure the robot to connect to: {0}:{1}".format(ip, port))
        processor.run()
        
    except ImportError as e:
        print("Error importing camera modules: {0}".format(e))
        print("Make sure you're running this on a PC with OpenCV and required libraries.")
        sys.exit(1)
    except Exception as e:
        print("Error running camera server: {0}".format(e))
        sys.exit(1)

def run_robot_controller(server_ip, server_port=5000):
    """Run the robot controller on EV3"""
    try:
        from robot.robot_controllers import RobotController
        
        print("Starting robot controller, connecting to server at {0}:{1}".format(server_ip, server_port))
        
        # Create and run robot controller
        controller = RobotController(server_ip, server_port)
        controller.run()
        
    except ImportError as e:
        print("Error importing robot modules: {0}".format(e))
        print("Make sure you're running this on an EV3 with ev3dev and required libraries.")
        sys.exit(1)
    except Exception as e:
        print("Error running robot controller: {0}".format(e))
        sys.exit(1)

def main():
    """Main function to parse arguments and run the appropriate mode"""
    parser = argparse.ArgumentParser(description="EV3 Robot Camera Navigation System")
    
    # Add common arguments
    parser.add_argument('--port', type=int, default=5000,
                      help='Port number for communication (default: 5000)')
    
    # Create subparsers for different modes
    subparsers = parser.add_subparsers(dest='mode', help='Operation mode')
    
    # Camera server mode
    camera_parser = subparsers.add_parser('camera', help='Run camera server (PC side)')
    camera_parser.add_argument('--model', type=str, default=None,
                             help='Path to YOLO model (default: src/camera/best.pt)')
    
    # Robot controller mode
    robot_parser = subparsers.add_parser('robot', help='Run robot controller (EV3 side)')
    robot_parser.add_argument('--server', type=str, required=False,
                            help='IP address of camera server')
    
    # Parse arguments
    args = parser.parse_args()
    
    # Determine mode based on arguments or platform
    mode = args.mode
    
    if mode is None:
        # Auto-detect mode based on platform
        if is_running_on_ev3():
            mode = 'robot'
            print("Detected EV3 platform, running in robot mode")
        else:
            mode = 'camera'
            print("Detected PC platform, running in camera server mode")
    
    # Run in selected mode
    if mode == 'camera':
        model_path = None
        if hasattr(args, 'model'):
            model_path = args.model
        run_camera_server(port=args.port, model_path=model_path)
    elif mode == 'robot':
        server_ip = args.server
        
        if server_ip is None:
            print("No server IP specified. Please provide the IP address of the camera server.")
            print("Example: python main.py robot --server 192.168.0.100")
            sys.exit(1)
            
        run_robot_controller(server_ip, args.port)

if __name__ == "__main__":
    main() 