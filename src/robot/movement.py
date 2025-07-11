# -*- coding: utf-8 -*-
"""
Time-based turning/forward movement, executes commands received from camera
"""

from time import sleep
from ..config.settings import *

# Simple movement methods are now in robot controller

# Executes commands from vision system
def execute_movement_command(robot_controller, command):
    try:
        cmd_type = command.get('command')  # Executes commands from vision system
        
        if cmd_type == "simple_turn":
            direction = command.get('direction', 'left')
            duration = command.get('duration', 0.5)
            # Convert duration to angle degrees
            # ESTIMATED_TURN_RATE is degrees per second, so angle = duration * rate
            angle_degrees = duration * ESTIMATED_TURN_RATE
            print("Converting turn: {:.3f}s duration -> {:.1f} degrees (ESTIMATED_TURN_RATE={})".format(
                duration, angle_degrees, ESTIMATED_TURN_RATE))
            robot_controller.simple_turn(direction, angle_degrees)
            
        elif cmd_type == "simple_turn_rotation":
            direction = command.get('direction', 'left')
            rotations = command.get('rotations', 0.5)
            print("Rotation-based turn: {} {:.3f} rotations".format(direction, rotations))
            robot_controller.simple_turn_rotation(direction, rotations)
            
        elif cmd_type == "simple_forward":
            distance = command.get('distance', 10)
            robot_controller.simple_forward(distance)
            
        elif cmd_type == "forward":
            distance = command.get('distance', 10)
            robot_controller.simple_forward(distance)
            

            
        elif cmd_type == "simple_backward":
            distance = command.get('distance', 10)
            robot_controller.simple_backward(distance)
            
        elif cmd_type == "turn_180":
            robot_controller.turn_180_degrees()
            
        elif cmd_type == "blind_ball_collection":
            robot_controller.blind_ball_collection()
            
        elif cmd_type == "release_balls":
            robot_controller.release_balls()
            
        elif cmd_type == "stop":
            robot_controller.stop_all_motors()
            print("Stop command executed")
            
        else:
            print("Unknown command: {}".format(cmd_type))
    
    except Exception as e:
        print("Error executing movement command: {}".format(e))
        robot_controller.stop_all_motors()

# COORDINATE NAVIGATION AS IN THE OLD FILE
def execute_coordinate_command(robot_controller, command):
    try:
        if 'coordinates' in command:
            coords = command['coordinates']
            print("\nReceived coordinates: {}".format(coords))
            
            # Coordinates come in mm, convert to cm for calculation
            target_x = coords['target_x'] / 10.0
            target_y = coords['target_y'] / 10.0  
            current_x = coords['current_x'] / 10.0
            current_y = coords['current_y'] / 10.0
            
            dx = target_x - current_x
            dy = target_y - current_y
            
            # Calculate target angle and normalize it
            import math
            target_angle = math.degrees(math.atan2(dy, dx))
            target_angle = robot_controller.normalize_angle(target_angle)
            
            # Calculate shortest turn from current angle
            current_angle = robot_controller.normalize_angle(robot_controller.get_current_heading())
            angle_diff = robot_controller.normalize_angle(target_angle - current_angle)
            
            print("\nNew navigation:")
            print("From position: ({:.1f}, {:.1f})".format(current_x, current_y))
            print("To position: ({:.1f}, {:.1f})".format(target_x, target_y))
            print("Current angle: {}, Target angle: {}, Need to turn: {}".format(
                current_angle, target_angle, angle_diff))
            
            # Turn towards target with simple turn
            if abs(angle_diff) > 5:  # Threshold for turning
                direction = "right" if angle_diff > 0 else "left"
                # Send direct angle in degrees instead of duration
                robot_controller.simple_turn(direction, abs(angle_diff))
            
            # Calculate distance and drive
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > 2:  # Minimum distance threshold
                print("Driving {:.1f} cm".format(distance))
                robot_controller.simple_forward(distance)
            
    except Exception as e:
        print("Error executing coordinate command: {}".format(e))
        robot_controller.stop_all_motors() 