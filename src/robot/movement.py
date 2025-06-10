# -*- coding: utf-8 -*-
"""
tidbaseret drejning/fremad, udfører kommandoer den får fra camera
"""

from time import sleep
from ..config.settings import *

# Simple movement metoder er nu i robot controller

#udfører kommandoer fra vision system med forbedret præcision
def execute_movement_command(robot_controller, command):
    try:
        cmd_type = command.get('command')  # Udfører kommandoer fra vision system
        
        if cmd_type == "simple_turn":
            direction = command.get('direction', 'left')
            duration = command.get('duration', 0.5)
            speed = command.get('speed', None)  # Understøt variabel hastighed
            angle_degrees = command.get('angle_degrees', None)  # NØGLE: Brug rotation baseret
            robot_controller.simple_turn(direction, duration, speed, angle_degrees)
            
        elif cmd_type == "precision_turn":
            direction = command.get('direction', 'left')
            angle_degrees = command.get('angle_degrees', 10)
            robot_controller.precision_turn(direction, angle_degrees)
            
        elif cmd_type == "simple_forward":
            distance = command.get('distance', 10)
            speed = command.get('speed', None)  # Understøt variabel hastighed
            robot_controller.simple_forward(distance, speed)
            
        elif cmd_type == "precision_forward":
            distance = command.get('distance', 10)
            robot_controller.precision_forward(distance)
            
        elif cmd_type == "forward":
            distance = command.get('distance', 10)
            robot_controller.simple_forward(distance)
            
        elif cmd_type == "start_harvester":
            robot_controller.start_harvester()
            print("Harvester started command executed")
            
        elif cmd_type == "stop_harvester":
            robot_controller.stop_harvester()
            print("Harvester stopped command executed")
            
        elif cmd_type == "stop":
            robot_controller.stop_all_motors()
            print("Stop command executed (all motors including harvester)")
            
        else:
            print("Unknown command: {}".format(cmd_type))
    
    except Exception as e:
        print("Error executing movement command: {}".format(e))
        robot_controller.stop_all_motors()

#FORBEDRET KOORDINAT NAVIGATION MED PROGRESSIV KORREKTION
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
            
            # Beregn afstand til mål
            distance = math.sqrt(dx*dx + dy*dy)
            
            print("\nProgressive navigation:")
            print("From position: ({:.1f}, {:.1f})".format(current_x, current_y))
            print("To position: ({:.1f}, {:.1f})".format(target_x, target_y))
            print("Current angle: {:.1f}°, Target angle: {:.1f}°, Need to turn: {:.1f}°".format(
                current_angle, target_angle, angle_diff))
            print("Distance: {:.1f} cm".format(distance))
            
            # PROGRESSIV DREJNING - forskellige thresholds baseret på afstand
            if distance > PRECISION_DISTANCE:
                # Langt væk: brug grov threshold til hurtig retning
                turn_threshold = COARSE_TURN_THRESHOLD
            elif distance > DISTANCE_THRESHOLD * 3:
                # Mellemafstand: brug fin threshold
                turn_threshold = FINE_TURN_THRESHOLD
            else:
                # Tæt på: brug meget fin threshold til præcis sigtning
                turn_threshold = VERY_FINE_TURN_THRESHOLD
            
            # Drej kun hvis nødvendigt baseret på afstand
            if abs(angle_diff) > turn_threshold:
                direction = "right" if angle_diff > 0 else "left"
                robot_controller.precision_turn(direction, abs(angle_diff))
            
            # Kør frem med progressiv hastighed
            elif distance > DISTANCE_THRESHOLD:
                actual_distance = robot_controller.precision_forward(distance)
                print("Drove {:.1f} cm towards target".format(actual_distance))
            
    except Exception as e:
        print("Error executing coordinate command: {}".format(e))
        robot_controller.stop_all_motors() 