# -*- coding: utf-8 -*-
"""
Beregner robot heading, target heading, vinkel forskel, afstand til mål, laver kommandoer
"""

import math
from ..config.settings import *

# Camera and object heights in cm
CAMERA_HEIGHT = 162.0  # Camera mounted at 162cm height above ground
ROBOT_HEIGHT = 20.0    # Robot markers are 20cm tall
BALL_HEIGHT = 4.0      # Balls are 4cm tall
GROUND_LEVEL = 0.0     # Reference level (ground)

# Camera parameters (these should be calibrated, but reasonable estimates)
IMAGE_WIDTH = 640     # Typical camera resolution
IMAGE_HEIGHT = 480
CAMERA_CENTER_X = IMAGE_WIDTH // 2   # Optical center X
CAMERA_CENTER_Y = IMAGE_HEIGHT // 2  # Optical center Y

def correct_position_to_ground_level(pos, object_height, camera_height):
    """
    Correct object position from its actual height to ground level
    pos: (x,y) pixel coordinates of object at its height
    object_height: height of object in cm
    camera_height: height of camera in cm
    Returns: (x,y) corrected coordinates as if object was at ground level
    """
    # Calculate distances from camera center (optical center)
    dx_from_center = pos[0] - CAMERA_CENTER_X
    dy_from_center = pos[1] - CAMERA_CENTER_Y
    
    # Distance from camera center in pixels
    radial_distance = math.sqrt(dx_from_center**2 + dy_from_center**2)
    
    if radial_distance < 1:  # At camera center, no correction needed
        return pos
    
    # Height difference from ground level
    height_diff = object_height - GROUND_LEVEL  # e.g., robot(20) - ground(0) = 20cm
    
    # The perspective correction is proportional to:
    # 1. Distance from camera center (more distortion at edges)
    # 2. Height above ground
    # 3. Inverse of camera height (closer camera = more distortion)
    
    # Correction factor: how much the height affects position
    # Objects higher than ground appear closer to camera center
    correction_magnitude = (height_diff / camera_height) * (radial_distance / 200.0)
    
    # Apply correction away from camera center
    # (higher objects appear closer to center, so correct outward to get ground position)
    if radial_distance > 0:
        correction_x = (dx_from_center / radial_distance) * correction_magnitude
        correction_y = (dy_from_center / radial_distance) * correction_magnitude
        
        # Apply correction - move AWAY from center to get ground position
        corrected_x = pos[0] + correction_x
        corrected_y = pos[1] + correction_y
        
        return (int(corrected_x), int(corrected_y))
    
    return pos

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