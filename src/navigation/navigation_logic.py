# -*- coding: utf-8 -*-
"""
Navigation logic for the robot - handles turning and forward commands
"""

import time

def handle_robot_navigation(navigation_info, commander, route_manager):
    """Main navigation logic - handles turning and forward movement"""
    if not navigation_info or not commander.can_send_command():
        return
    
    angle_diff = navigation_info["angle_diff"]
    distance_cm = navigation_info["distance_cm"]
    
    print("Navigation: Target={} Angle diff={:.1f}deg, Distance={:.1f}cm".format(
        "Available", angle_diff, distance_cm))
    
    # Udvidet hitting zone: ±3 grader
    hitting_zone_min = -3.0
    hitting_zone_max = 3.0
    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
    
    
    
    if not in_hitting_zone:
        return handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager)
    elif distance_cm > 22:
        return handle_forward_movement(distance_cm, angle_diff, commander)
    else:
        return handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone, commander, route_manager)

def handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager):
    """Handles angle correction"""
    # Check if we have tried too many times on this target
    if route_manager.should_skip_current_target():
        print("TOO MANY ATTEMPTS ON THIS TARGET - SKIPPING")
        route_manager.advance_to_next_target()
        return
    
    direction = "right" if angle_diff > hitting_zone_max else "left"
    
    # Calculate how much to turn to get into hitting zone
    if angle_diff > hitting_zone_max:
        turn_amount = angle_diff - hitting_zone_max  # Turn to max hitting zone
    else:  # angle_diff < hitting_zone_min
        turn_amount = hitting_zone_min - angle_diff  # Turn to min hitting zone
    
    # CORRECTED ROTATION: 180° = 0.5 rotations 
    rotations = turn_amount / 180.0 * 0.5
    
    # MINIMUM ROTATION: Round small rotations up to 0.01 to ensure motor movement
    min_rotation = 0.01
    if rotations < min_rotation:
        print("WARNING: Rotation {:.6f} too small - rounded up to {:.3f}".format(rotations, min_rotation))
        rotations = min_rotation
    
    # LIMIT ROTATION: Smaller rotations for fine-tuning
    # Large adjustments first, then fine adjustments
    # if abs(angle_diff) > 15.0:
    #     max_rotations = 0.20  # Large corrections
    # elif abs(angle_diff) > 5.0:
    #     max_rotations = 0.10  # Medium corrections  
    # else:
    #     max_rotations = 0.05  # Fine adjustments
        
    # if rotations > max_rotations:
    #     rotations = max_rotations
    #     print("WARNING: Rotation limited to {:.3f} for precision (was {:.3f})".format(max_rotations, turn_amount / 180.0 * 0.5))
    
    print("ANGLE CORRECTION: {:.1f}deg -> hitting zone [{:.1f}, {:.1f}] -> {:.3f} rotations".format(
        angle_diff, hitting_zone_min, hitting_zone_max, rotations))
    commander.send_turn_rotation_command(direction, rotations)

def handle_forward_movement(distance_cm, angle_diff, commander):
    """Handles careful forward movement"""
    # Adaptive distance based on proximity to target
    remaining_distance = distance_cm - 22
    
    if remaining_distance <= 0:
        move_distance = 0 # Stop if already at or past the target
    elif remaining_distance <= 5:
        move_distance = 1.0 # Længere burst tæt på
    elif remaining_distance <= 30:
        move_distance = 4.0 # Længere burst mellemafstand
    else:
        move_distance = min(remaining_distance * 0.3, 10.0) # 30% af resten, max 10cm
    if move_distance < 1.0 and remaining_distance > 0:
        move_distance = 1.0
    print("IN HITTING ZONE - CAREFUL FORWARD {:.1f} cm (distance:{:.1f}cm, angle:{:.1f}deg) [Wall approach active]".format(
        move_distance, distance_cm, angle_diff))
    commander.send_forward_command(move_distance)

def handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, 
                          in_hitting_zone, commander, route_manager):
    """Handles ball collection when the robot is close enough"""
    print("=== READY FOR BLIND COLLECTION ===")
    print("Distance: {:.1f}cm <= 22cm, Angle: {:.1f}deg".format(distance_cm, angle_diff))
    print("Hitting zone: [{:.1f}, {:.1f}], In zone: {}".format(
        hitting_zone_min, hitting_zone_max, in_hitting_zone))
    print("Robot position good: distance <= 22cm AND in hitting zone")
    
    if in_hitting_zone:
        print("*** EXECUTING BLIND BALL COLLECTION ***")
        success = commander.send_blind_ball_collection_command()
        if success:
            print("Blind collection command sent successfully!")
            # GO TO NEXT POINT IN ROUTE after collection
            route_manager.advance_to_next_target()
            return True # Return True on successful command send
        else:
            print("Failed to send blind collection command!")
            # Increase number of attempts and check if we should give up
            should_skip = route_manager.increment_collection_attempts()
            if should_skip:
                print("MAX ATTEMPTS REACHED - SKIPPING TO NEXT TARGET")
                route_manager.advance_to_next_target()
            return False # Return False if command failed to send
    else:
        print("NOT IN HITTING ZONE - NEED ANGLE ADJUSTMENT FIRST")
        print("   Angle {:.1f}° is outside [{:.1f}°, {:.1f}°]".format(
            angle_diff, hitting_zone_min, hitting_zone_max))
        return False # Not in hitting zone, no collection attempted