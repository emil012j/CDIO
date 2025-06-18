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

# Camera parameters
CAMERA_FOV_DEGREES = 90.0  # Field of view in degrees

# Get camera parameters from actual camera resolution
def get_camera_center():
    """Get camera center from current camera resolution"""
    from ..config.settings import CAMERA_RESOLUTION
    width, height = CAMERA_RESOLUTION
    return width // 2, height // 2

def correct_position_to_ground_level(pos, object_height, camera_height):
    """
    PRECISE perspective correction using camera FOV and real-world geometry
    
    Uses actual camera FOV (70°) to calculate true ground-plane projection.
    Accounts for non-linear perspective distortion across the image.
    
    pos: (x,y) pixel coordinates of object at its height
    object_height: height of object in cm  
    camera_height: height of camera in cm
    Returns: (x,y) corrected coordinates as if object was at ground level
    """
    # Get actual camera center and resolution
    camera_center_x, camera_center_y = get_camera_center()
    from ..config.settings import CAMERA_RESOLUTION
    image_width, image_height = CAMERA_RESOLUTION
    
    # Calculate distance from camera center in pixels
    dx_from_center = pos[0] - camera_center_x
    dy_from_center = pos[1] - camera_center_y
    
    # Convert FOV to radians
    fov_rad = math.radians(CAMERA_FOV_DEGREES)
    
    # Calculate the real-world distance per pixel at the image edge
    # For a camera at height H with FOV θ, the width of ground covered is: 2 * H * tan(θ/2)
    ground_width_covered = 2 * camera_height * math.tan(fov_rad / 2)
    pixels_per_cm = image_width / ground_width_covered
    
    # Convert pixel distances to real-world distances (cm)
    real_world_dx = dx_from_center / pixels_per_cm
    real_world_dy = dy_from_center / pixels_per_cm
    
    # Calculate the viewing angle from camera center to this point
    distance_from_center = math.sqrt(real_world_dx**2 + real_world_dy**2)
    
    if distance_from_center < 0.1:  # At camera center, no correction needed
        return pos
    
    # Calculate viewing angle (how far off-axis this point is)
    viewing_angle = math.atan(distance_from_center / camera_height)
    
    # Height difference affects apparent position based on viewing geometry
    height_diff = object_height - GROUND_LEVEL
    
    # For perspective projection: apparent_distance = real_distance + height_offset
    # height_offset = height_diff * tan(viewing_angle)
    height_offset = height_diff * math.tan(viewing_angle)
    
    # Calculate corrected real-world position
    correction_factor = height_offset / distance_from_center if distance_from_center > 0 else 0
    corrected_real_dx = real_world_dx * (1 - correction_factor)
    corrected_real_dy = real_world_dy * (1 - correction_factor)
    
    # Convert back to pixel coordinates
    corrected_dx_pixels = corrected_real_dx * pixels_per_cm
    corrected_dy_pixels = corrected_real_dy * pixels_per_cm
    
    corrected_x = camera_center_x + corrected_dx_pixels
    corrected_y = camera_center_y + corrected_dy_pixels
    
    # Debug output for significant corrections
    correction_distance = math.sqrt((corrected_x - pos[0])**2 + (corrected_y - pos[1])**2)
    if correction_distance > 5:  # Log corrections > 5 pixels
        print(f"FOV CORRECTION: {object_height}cm object: {pos} -> ({int(corrected_x)},{int(corrected_y)})")
        print(f"  Real distance: {distance_from_center:.1f}cm, Viewing angle: {math.degrees(viewing_angle):.1f}°")
        print(f"  Height offset: {height_offset:.1f}cm, Correction: {correction_distance:.1f}px")
    
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