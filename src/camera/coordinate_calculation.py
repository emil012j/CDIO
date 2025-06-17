# -*- coding: utf-8 -*-
"""
Beregner robot heading, target heading, vinkel forskel, afstand til mål, laver kommandoer
"""

import math
from ..config.settings import *

# Camera and object heights in cm  
CAMERA_HEIGHT = 162.0   # Camera mounted at 162cm height above ground
ROBOT_HEIGHT = 20.0     # Robot markers are 20cm tall  
BALL_HEIGHT = 4.0       # Balls are 4cm tall
GROUND_LEVEL = 0.0      # Reference level (ground)

# Get camera parameters from actual camera resolution
def get_camera_center():
    """Get camera center from current camera resolution"""
    from ..config.settings import CAMERA_RESOLUTION
    width, height = CAMERA_RESOLUTION
    return width // 2, height // 2

def correct_position_to_ground_level(pos, object_height, camera_height):
    """
    SIMPLE geometric perspective correction for height difference
    
    When camera looks down, higher objects appear further from center than their ground position.
    
    For an object at height H viewed by camera at height C:
    - Ground position factor = (C - H) / C
    - Robot (20cm): factor = (162-20)/162 = 0.877  
    - Ball (4cm): factor = (162-4)/162 = 0.975
    
    pos: (x,y) pixel coordinates of object at its height
    object_height: height of object in cm  
    camera_height: height of camera in cm
    Returns: (x,y) corrected coordinates as if object was at ground level
    """
    # Get actual camera center
    camera_center_x, camera_center_y = get_camera_center()
    
    # Calculate distance from camera center
    dx_from_center = pos[0] - camera_center_x
    dy_from_center = pos[1] - camera_center_y
    
    # Simple geometric correction factor
    # Ground position is closer to center than apparent position
    ground_factor = (camera_height - object_height) / camera_height
    
    # Apply correction - move position closer to center
    corrected_x = camera_center_x + (dx_from_center * ground_factor)
    corrected_y = camera_center_y + (dy_from_center * ground_factor)
    
    # Debug output for significant corrections
    correction_distance = math.sqrt((corrected_x - pos[0])**2 + (corrected_y - pos[1])**2)
    if correction_distance > 5:  # Log corrections > 5 pixels
        print(f"HEIGHT CORRECTION: {object_height}cm object: {pos} -> ({int(corrected_x)},{int(corrected_y)}) [factor={ground_factor:.3f}]")
    
    return (int(corrected_x), int(corrected_y))

#beregner afstand mellem to punkter
def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

#beregner vinklen mellem to punkter (OpenCV koordinat system - Y er vendt)
def calculate_angle_from_positions(from_pos, to_pos):
    dx = to_pos[0] - from_pos[0]
    dy = from_pos[1] - to_pos[1]  # Omvendt pga OpenCV koordinat system
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

#beregner robot heading (retning fra tail til head) ved at bruge vinklen mellem to punkter, feks. 0 grader er retningen til højre, 90 grader er retningen opad.
def calculate_robot_heading(robot_head, robot_tail):
    if not robot_head or not robot_tail:
        return None
    
    # Beregner robot heading (retning fra tail til head)
    head_pos = robot_head["pos"]
    tail_pos = robot_tail["pos"]
    return calculate_angle_from_positions(tail_pos, head_pos)

#beregner vinkel forskel mellem to vinkler. Hvis vinklen er 0, så peger robotten i retningen af målet.
def calculate_angle_difference(current_angle, target_angle):
    diff = target_angle - current_angle
    
    # Normalize to [-180, 180]
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    
    return diff

#beregner navigation kommandoer baseret på robotens position og målet, til at bevæge sig mod målet.
def calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor):
    if not robot_head or not robot_tail or not target_ball:
        return None
    
    # Correct ALL positions to ground level for consistent calculation
    corrected_head = correct_position_to_ground_level(
        robot_head["pos"], ROBOT_HEIGHT, CAMERA_HEIGHT
    )
    corrected_tail = correct_position_to_ground_level(
        robot_tail["pos"], ROBOT_HEIGHT, CAMERA_HEIGHT  
    )
    corrected_ball = correct_position_to_ground_level(
        target_ball, BALL_HEIGHT, CAMERA_HEIGHT
    )
    
    # Calculate robot center using corrected positions
    robot_center = (
        (corrected_head[0] + corrected_tail[0]) // 2,
        (corrected_head[1] + corrected_tail[1]) // 2
    )
    
    # Calculate robot heading using corrected head/tail positions
    robot_heading = calculate_angle_from_positions(corrected_tail, corrected_head)
    if robot_heading is None:
        return None
    
    # Calculate target heading using corrected positions
    target_heading = calculate_angle_from_positions(robot_center, corrected_ball)
    
    # Calculate angle difference
    angle_diff = calculate_angle_difference(robot_heading, target_heading)
    
    # Calculate distance using corrected positions
    distance_pixels = calculate_distance(robot_center, corrected_ball)
    distance_cm = (distance_pixels * scale_factor) / 10.0 if scale_factor else distance_pixels / 10.0
    
    return {
        "robot_center": robot_center,
        "robot_heading": robot_heading,
        "target_heading": target_heading,
        "angle_diff": angle_diff,
        "distance_cm": distance_cm,
        "original_target": target_ball,  # Keep original for visualization
        "corrected_target": corrected_ball,  # Add corrected for debugging
        "corrected_head": corrected_head,    # For debugging
        "corrected_tail": corrected_tail     # For debugging
    }

#laver turn kommandoer baseret på vinkel forskel
def create_turn_command(angle_diff):
    if abs(angle_diff) <= TURN_THRESHOLD:
        return None
    
    # Laver turn kommandoer - ingen begrænsning, drej præcist!
    turn_amount = angle_diff
    
    return {
        "command": "simple_turn",
        "direction": "right" if turn_amount > 0 else "left",
        "duration": abs(turn_amount) / ESTIMATED_TURN_RATE
    }

#laver forward kommandoer baseret på afstand
def create_forward_command(distance_cm):
    if distance_cm <= DISTANCE_THRESHOLD:
        return None
    
    # Laver forward kommandoer
    move_distance = min(distance_cm, MAX_FORWARD_DISTANCE)
    
    return {
        "command": "forward",
        "distance": move_distance
    } 