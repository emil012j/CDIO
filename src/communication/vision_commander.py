# -*- coding: utf-8 -*-
"""
Handles network connection (TCP socket), sends JSON commands (e.g. turning, forward, stop as JSON), 
rate limiting (not too many commands per second, as the robot cannot handle them), 
timeout/error handling (if the robot doesn't respond)
"""

import socket
import json
import time
from ..config.settings import *

# Handles network connection, sends JSON commands, rate limiting, timeout/error handling
class VisionCommander:
    # Initializes robot IP and port
    def __init__(self, robot_ip=ROBOT_IP, command_port=COMMAND_PORT):
        self.robot_ip = robot_ip
        self.command_port = command_port
        self.last_command_time = 0
        
    # Checks if a certain time has passed since the last command
    def can_send_command(self):
        current_time = time.time()
        return current_time - self.last_command_time >= COMMAND_COOLDOWN  # Rate limiting (not too many commands per second)
    
    # Sends JSON commands over TCP socket
    def send_command(self, command_dict):
        if not self.can_send_command():
            return False
            
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Handles network connection to EV3
            sock.settimeout(0.5)  # Longer timeout to allow robot processing
            sock.connect((self.robot_ip, self.command_port))
            
            # Sends JSON commands over TCP socket
            command_json = json.dumps(command_dict).encode()
            sock.send(command_json)
            sock.close()
            
            self.last_command_time = time.time()
            return True
            
        except socket.timeout:
            print("Timeout connecting to robot")
            return False
        except ConnectionRefusedError:
            print("Robot refused connection - check if robot is running")
            return False
        except Exception as e:
            print("Error sending command: {}".format(e))
            return False
    
    # Sends a turn command by sending a JSON with the command, direction and time. e.g. {"command": "simple_turn", "direction": "left", "duration": 1.0}
    def send_turn_command(self, direction, duration):
        """Send a turn command to the robot"""
        command = {
            "command": "simple_turn",
            "direction": direction,
            "duration": duration
        }
        return self.send_command(command)
    
    # Sends a turn command based on rotations (rotation-based)
    def send_turn_rotation_command(self, direction, rotations):
        """Send a rotation-based turn command to the robot"""
        command = {
            "command": "simple_turn_rotation",
            "direction": direction,
            "rotations": rotations
        }
        return self.send_command(command)
    
    # Sends a forward command by sending a JSON with the command and distance. e.g. {"command": "forward", "distance": 10}
    def send_forward_command(self, distance):
        """Send a forward movement command to the robot"""
        command = {
            "command": "forward", 
            "distance": distance
        }
        return self.send_command(command)
    
    # Sends a precise forward command WITHOUT overshoot for fine positioning
    def send_forward_precise_command(self, distance):
        """Send a precise forward movement command (no overshoot) to the robot"""
        command = {
            "command": "forward_precise", 
            "distance": distance
        }
        return self.send_command(command)
    
    # Sends a stop command by sending a JSON with the command. e.g. {"command": "stop"}
    def send_stop_command(self):
        """Send a stop command to the robot"""
        command = {
            "command": "stop"
        }
        return self.send_command(command)
    
    # Sends a backward command by sending a JSON with the command and distance. e.g. {"command": "simple_backward", "distance": 10}
    def send_backward_command(self, distance):
        """Send a backward movement command to the robot"""
        command = {
            "command": "simple_backward",
            "distance": distance
        }
        return self.send_command(command)
    
    # Sends a 180 degree turn command by sending a JSON with the command. e.g. {"command": "turn_180"}
    def send_turn_180_command(self):
        """Send a 180 degree turn command to the robot"""
        command = {
            "command": "turn_180"
        }
        return self.send_command(command)
    
    # Sends blind ball collection command - drives forward, backs up, turns 180
    def send_blind_ball_collection_command(self):
        """Send blind ball collection sequence command to the robot"""
        command = {
            "command": "blind_ball_collection"
        }
        return self.send_command(command) 
    
    def send_release_balls_command(self, duration):
        """Send a command to release balls (run collect motor backwards)"""
        command = {
            "command": "release_balls",
            "duration": duration
        }
        return self.send_command(command)

    def send_continuous_move_command(self, speed):
        """Send a continuous movement command to the robot (forward/backward at given speed)"""
        command = {
            "command": "continuous_move",
            "speed": speed
        }
        return self.send_command(command)