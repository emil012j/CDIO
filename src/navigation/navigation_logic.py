# -*- coding: utf-8 -*-
"""
Simplified Navigation logik for robotten - align once, then drive exact distance to target
"""

import time

def handle_robot_navigation(navigation_info, commander, route_manager):
    """Simple navigation logic - align once, then drive exact distance to target"""
    if not navigation_info or not commander.can_send_command():
        return
    
    angle_diff = navigation_info["angle_diff"]
    distance_cm = navigation_info["distance_cm"]
    
    print("Navigation: Angle diff={:.1f}deg, Distance={:.1f}cm".format(angle_diff, distance_cm))
    
    # Check if we should skip this target after too many attempts
    if route_manager.should_skip_current_target():
        print("🚫 TOO MANY ATTEMPTS ON THIS TARGET - SKIPPING")
        route_manager.advance_to_next_target()
        return
    
    # Check if we've reached the target (ball will be collected automatically)
    if distance_cm <= 5:  # Very close to target
        print("🎯 REACHED TARGET - Ball should be collected!")
        route_manager.advance_to_next_target()
        return
    
    # ANGLE CORRECTION: Only correct if significantly off course (>3°)
    if abs(angle_diff) > 3.0:
        handle_angle_correction(angle_diff, commander)
        return
    
    # DRIVE TO TARGET: Drive the exact distance to target
    print("DRIVING EXACT DISTANCE: {:.1f}cm to target".format(distance_cm))
    commander.send_forward_command(distance_cm)

def handle_angle_correction(angle_diff, commander):
    """Simple angle correction - turn the full amount needed"""
    direction = "right" if angle_diff > 0 else "left"
    turn_amount = abs(angle_diff)
    
    # Convert to rotations: 180° = 0.5 rotations
    rotations = turn_amount / 180.0 * 0.5
    
    # Minimum rotation to ensure motor movement
    if rotations < 0.01:
        rotations = 0.01
    
    # Maximum rotation for safety
    if rotations > 0.25:
        rotations = 0.25
        print("WARNING: Large rotation capped at 0.25 (was {:.3f})".format(turn_amount / 180.0 * 0.5))
    
    print("ANGLE CORRECTION: {:.1f}deg -> {:.3f} rotations {}".format(
        turn_amount, rotations, direction))
    commander.send_turn_rotation_command(direction, rotations)