# -*- coding: utf-8 -*-
"""
Beregner robot heading, target heading, vinkel forskel, afstand til mål, laver kommandoer
"""

import math
from ..config.settings import *

# Camera and object heights in cm
CAMERA_HEIGHT = 162.0  # Camera mounted at 162cm height
ROBOT_HEIGHT = 20.0   # Robot is 20cm tall
BALL_HEIGHT = 4.0    # Balls are 4cm tall

def correct_for_height_difference(pos1, pos2, height1, height2, camera_height):
    """
    Correct for height difference between two objects in camera view
    pos1, pos2: (x,y) pixel coordinates
    height1, height2: heights of objects in cm
    camera_height: height of camera in cm
    Returns: (x,y) corrected coordinates for pos2
    """
    # Calculate distance from camera to each point in pixels
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    pixel_distance = math.sqrt(dx*dx + dy*dy)
    
    if pixel_distance < 1:  # Too close to calculate
        return pos2
        
    # Calculate height difference ratio
    height_diff = height2 - height1
    height_ratio = height_diff / camera_height
    
    # Calculate correction vector
    # The correction is proportional to distance and height difference
    correction_factor = pixel_distance * height_ratio
    correction_x = dx * correction_factor
    correction_y = dy * correction_factor
    
    # Apply correction
    corrected_x = pos2[0] - correction_x
    corrected_y = pos2[1] - correction_y
    
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
    
    # Calculate robot center
    robot_center = (
        (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
        (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    )
    
    # Correct target position for height difference
    corrected_target = correct_for_height_difference(
        robot_center, 
        target_ball,
        ROBOT_HEIGHT,
        BALL_HEIGHT,
        CAMERA_HEIGHT
    )
    
    robot_heading = calculate_robot_heading(robot_head, robot_tail)
    if robot_heading is None:
        return None
    
    # Calculate target heading using corrected position
    target_heading = calculate_angle_from_positions(robot_center, corrected_target)
    
    # Calculate angle difference
    angle_diff = calculate_angle_difference(robot_heading, target_heading)
    
    # Calculate distance using corrected position
    distance_pixels = calculate_distance(robot_center, corrected_target)
    distance_cm = (distance_pixels * scale_factor) / 10.0 if scale_factor else distance_pixels / 10.0
    
    return {
        "robot_center": robot_center,
        "robot_heading": robot_heading,
        "target_heading": target_heading,
        "angle_diff": angle_diff,
        "distance_cm": distance_cm,
        "original_target": target_ball,  # Keep original for visualization
        "corrected_target": corrected_target  # Add corrected for debugging
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