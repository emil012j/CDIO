# -*- coding: utf-8 -*-
"""
Beregner robot heading, target heading, vinkel forskel, afstand til mål, laver kommandoer
"""

import math
from ..config.settings import *

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
    distance_cm = (distance_pixels * scale_factor) / 10.0 if scale_factor else distance_pixels / 10.0
    
    return {
        "robot_center": robot_center,
        "robot_heading": robot_heading,
        "target_heading": target_heading,
        "angle_diff": angle_diff,
        "distance_cm": distance_cm
    }

#laver turn kommandoer baseret på vinkel forskel med progressiv præcision
def create_turn_command(angle_diff, distance_to_target=None):
    # Bestem threshold baseret på afstand til mål (hvis tilgængelig)
    if distance_to_target is not None:
        if distance_to_target > PRECISION_DISTANCE:
            threshold = COARSE_TURN_THRESHOLD
            max_turn = MAX_TURN_COARSE
        elif distance_to_target > DISTANCE_THRESHOLD * 3:
            threshold = FINE_TURN_THRESHOLD
            max_turn = MAX_TURN_FINE
        else:
            threshold = VERY_FINE_TURN_THRESHOLD
            max_turn = MAX_TURN_PRECISION
    else:
        # Default til fin justering hvis afstand ikke er kendt
        threshold = FINE_TURN_THRESHOLD
        max_turn = MAX_TURN_FINE
    
    if abs(angle_diff) <= threshold:
        return None
    
    # Begræns drejning baseret på afstand til mål
    turn_amount = max(-max_turn, min(max_turn, angle_diff))
    
    return {
        "command": "precision_turn",
        "direction": "right" if turn_amount > 0 else "left",
        "angle_degrees": abs(turn_amount)
    }

#laver forward kommandoer baseret på afstand med progressiv hastighed
def create_forward_command(distance_cm):
    if distance_cm <= DISTANCE_THRESHOLD:
        return None
    
    # Brug precision forward til intelligent hastighedskontrol
    return {
        "command": "precision_forward",
        "distance": distance_cm
    }

#laver optimeret turn kommando der tager højde for både vinkel og afstand
def create_optimized_turn_command(angle_diff, distance_to_target):
    """Skaber turn kommando optimeret til boldtargeting"""
    return create_turn_command(angle_diff, distance_to_target)

#laver optimeret forward kommando med afstandsbaseret hastighed
def create_optimized_forward_command(distance_cm, angle_accuracy):
    """Skaber forward kommando med hastighed baseret på hvor godt retningen passer"""
    if distance_cm <= DISTANCE_THRESHOLD:
        return None
    
    # Hvis retningen ikke er helt korrekt, kør kun kort distance
    if abs(angle_accuracy) > VERY_FINE_TURN_THRESHOLD:
        max_distance = MAX_FORWARD_DISTANCE_CLOSE
    elif distance_cm > PRECISION_DISTANCE:
        max_distance = MAX_FORWARD_DISTANCE_FAR
    else:
        max_distance = MAX_FORWARD_DISTANCE_NEAR
    
    move_distance = min(distance_cm, max_distance)
    
    return {
        "command": "precision_forward",
        "distance": move_distance
    } 