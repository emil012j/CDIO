# -*- coding: utf-8 -*-
"""
Beregner robot heading, target heading, vinkel forskel, afstand til mål, laver kommandoer
"""

import math
from ..config.settings import *
from ..camera.detection import *


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
def calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor, cross_pos):
    if not robot_head or not robot_tail or not target_ball:
        return None
    
    #beregner robotens midtpunkt og heading
    robot_center = (
        (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
        (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    )
    
    robot_heading = calculate_robot_heading(robot_head, robot_tail)
    if robot_heading is None:
        return None
    
    # Beregner target heading (retning robot til bold)
    target_heading = calculate_angle_from_positions(robot_center, target_ball)
    
    # Beregner vinkel forskel (hvor meget skal robotten dreje?)
    angle_diff = calculate_angle_difference(robot_heading, target_heading)
    
    # Beregner afstand til mål
    distance_pixels = calculate_distance(robot_center, target_ball)
    
    # Sikrer scale_factor er en float
    if isinstance(scale_factor, (int, float)) and scale_factor > 0:
        distance_cm = (distance_pixels * scale_factor) / 10.0
    else:
        print(f"[ADVARSEL] scale_factor={scale_factor} er ikke brugbar. Bruger fallback.")
        distance_cm = distance_pixels / 10.0
    
    # Hvis der er et kryds, så undgå det
    if isinstance(cross_pos, tuple) and len(cross_pos) == 2:
        cross_distance = calculate_distance(robot_center, cross_pos)
    else:
        cross_distance = float("inf")
        
    avoid_cross = cross_distance < CROSS_AVOID_RADIUS

    return {
        "robot_center": robot_center,
        "robot_heading": robot_heading,
        "target_heading": target_heading,
        "angle_diff": angle_diff,
        "distance_cm": distance_cm,
        "avoid_cross": avoid_cross
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