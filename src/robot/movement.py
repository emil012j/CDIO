# -*- coding: utf-8 -*-
"""
tidbaseret drejning/fremad, udfører kommandoer den får fra camera
"""


import time
from time import sleep
from ..config.settings import *

# Simple movement metoder er nu i robot controller

#udfører kommandoer fra vision system
def execute_movement_command(robot_controller, command):
    try:
        cmd_type = command.get('command')  # Udfører kommandoer fra vision system
        
        if cmd_type == "simple_turn":
            direction = command.get('direction', 'left')
            duration = command.get('duration', 0.5)
            # Konverter duration til vinkel grader
            # ESTIMATED_TURN_RATE er grader per sekund, så angle = duration * rate
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
            
        elif cmd_type == "stop":
            robot_controller.stop_all_motors()
            print("Stop command executed")
            
        else:
            print("Unknown command: {}".format(cmd_type))
    
    except Exception as e:
        print("Error executing movement command: {}".format(e))
        robot_controller.stop_all_motors()

#KOORDINAT NAVIGATION SOM I DEN GAMLE FIL
def execute_coordinate_command(robot_controller, command):
    try:
        if 'coordinates' in command:
            coords = command['coordinates']
            print("\nReceived coordinates: {}".format(coords))
            
            # Koordinater kommer i mm, konverter til cm for beregning
            target_x = coords['target_x'] / 10.0
            target_y = coords['target_y'] / 10.0  
            current_x = coords['current_x'] / 10.0
            current_y = coords['current_y'] / 10.0
            
            dx = target_x - current_x
            dy = target_y - current_y
            
            # Beregn target vinkel og normaliser den
            import math
            target_angle = math.degrees(math.atan2(dy, dx))
            target_angle = robot_controller.normalize_angle(target_angle)
            
            # Beregn korteste drejning fra nuværende vinkel
            current_angle = robot_controller.normalize_angle(robot_controller.get_current_heading())
            angle_diff = robot_controller.normalize_angle(target_angle - current_angle)
            
            print("\nNew navigation:")
            print("From position: ({:.1f}, {:.1f})".format(current_x, current_y))
            print("To position: ({:.1f}, {:.1f})".format(target_x, target_y))
            print("Current angle: {}, Target angle: {}, Need to turn: {}".format(
                current_angle, target_angle, angle_diff))
            
            # Drej mod målet med simple turn
            if abs(angle_diff) > 5:  # Threshold for drejning
                direction = "right" if angle_diff > 0 else "left"
                # Send direkte vinkel i grader i stedet for duration
                robot_controller.simple_turn(direction, abs(angle_diff))
            
            # Beregn afstand og kør
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > 2:  # Minimum afstand threshold
                print("Driving {:.1f} cm".format(distance))
                robot_controller.simple_forward(distance)
            
    except Exception as e:
        print("Error executing coordinate command: {}".format(e))
        robot_controller.stop_all_motors() 