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
    
    # Check if current target is a waypoint (safe spot) or actual target (ball/goal)
    is_waypoint = route_manager.is_current_target_waypoint()
    target_type = "WAYPOINT" if is_waypoint else "TARGET"
    
    print("Navigation: Target={} Angle diff={:.1f}deg, Distance={:.1f}cm".format(
        target_type, angle_diff, distance_cm))
    
    # PRECISE HITTING ZONE: Looser zone for waypoints, tighter for targets
    hitting_zone_min = -5.0 if is_waypoint else -1.0  # Looser for waypoints
    hitting_zone_max = 5.0 if is_waypoint else 1.0    # Looser for waypoints
    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
    
    # Different distance thresholds for waypoints vs targets  
    approach_distance = 25 if is_waypoint else 22  # 25cm acceptance zone for waypoints
    
    # DEBUG: Show current state vs thresholds every frame
    print("  DEBUG: Distance={:.1f}cm (≤{:.1f}cm?), Angle={:.1f}° in [{:.1f}°,{:.1f}°]? = {}".format(
        distance_cm, approach_distance, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone))
    
    # TURN PHASE: Correct angle if not in hitting zone
    if not in_hitting_zone:
        return handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager)
    
    # FORWARD PHASE: Careful forward movement when in hitting zone
    elif distance_cm > approach_distance:
        return handle_forward_movement(distance_cm, angle_diff, commander)
    
    # ARRIVAL HANDLING: Different behavior for waypoints vs targets
    else:
        if is_waypoint:
            return handle_waypoint_arrival(distance_cm, angle_diff, commander, route_manager)
        else:
            return handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max,
                                        in_hitting_zone, commander, route_manager)

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
        move_distance = 0.5 # Very small steps when very close
    elif remaining_distance <= 30:
        move_distance = 2.0 # Steps for 'close' range
    else:
        # Larger steps when further away, but capped to avoid overshooting
        move_distance = min(remaining_distance * 0.2, 5.0) # Move 20% of remaining distance, max 5cm
    
    # Ensure a minimum movement if still far from target to prevent getting stuck
    if move_distance < 0.5 and remaining_distance > 0:
        move_distance = 0.5
    
    print("IN HITTING ZONE - CAREFUL FORWARD {:.1f} cm (distance:{:.1f}cm, angle:{:.1f}deg) [Wall approach active]".format(
        move_distance, distance_cm, angle_diff))
    
    # Use normal forward command (forward_precise doesn't exist on robot)
    commander.send_forward_command(move_distance)

def handle_waypoint_arrival(distance_cm, angle_diff, commander, route_manager):
    """Handle arrival at a safe spot waypoint - 25cm diameter acceptance zone"""
    print(f"🛡️ WAYPOINT ARRIVAL: Distance={distance_cm:.1f}cm, Angle={angle_diff:.1f}°")
    
    # If still outside 12.5cm radius (25cm diameter), continue approaching
    if distance_cm > 12.5:  # 25cm diameter = 12.5cm radius
        print(f"🛡️ APPROACHING WAYPOINT: Moving {distance_cm:.1f}cm closer")
        # Move carefully towards waypoint center
        move_distance = min(distance_cm * 0.4, 8.0)  # Move 40% of distance, max 8cm
        commander.send_forward_command(move_distance)
        return False  # Not yet reached
    
    # Inside 25cm diameter circle - advance to next target
    print("🛡️ *** REACHED WAYPOINT (within 25cm diameter) - ADVANCING TO NEXT TARGET ***")
    route_manager.advance_to_next_target()
    return True  # Successfully reached waypoint

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