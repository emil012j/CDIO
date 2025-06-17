# -*- coding: utf-8 -*-
"""
Navigation logik for robotten - håndterer turning og forward kommandoer
"""

import time
from src.communication.vision_commander import VisionCommander


def handle_robot_navigation(navigation_info, commander, route_manager):
    """Hovednavigations logik - håndterer drejning og fremadkørsel"""
    if not navigation_info or not commander.can_send_command():
        return
    
    angle_diff = navigation_info["angle_diff"]
    distance_cm = navigation_info["distance_cm"]
    
    print("Navigation: Target={} Angle diff={:.1f}deg, Distance={:.1f}cm".format(
        "Available", angle_diff, distance_cm))
    
    # PRECISE HITTING ZONE: Strammere zone for bedre præcision
    hitting_zone_min = -1.0  # Strammere: Minimum præcis angle for at ramme bolden
    hitting_zone_max = 1.0   # Strammere: Maximum præcis angle for at ramme bolden
    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
    
    # DEBUG: Show current state vs thresholds every frame
    print("  DEBUG: Distance={:.1f}cm (≤22cm?), Angle={:.1f}° in [{:.1f}°,{:.1f}°]? = {}".format(
        distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone))
    
    # TURN PHASE: Korriger vinkel hvis ikke i hitting zone
    if not in_hitting_zone:
        return handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager)
    
    # FORWARD PHASE: Forsigtig fremadkørsel når i hitting zone
    elif distance_cm > 25:  # Stop når vi er 22 cm væk for blind collection
        return handle_forward_movement(distance_cm, angle_diff, commander)
    
    # BLIND BALL COLLECTION: I hitting zone OG ≤22 cm væk - start blind collection
    else:
        return handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, 
                                    in_hitting_zone, commander, route_manager)

def handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager):
    """Håndterer vinkel korrektion"""
    # Tjek om vi har prøvet for mange gange på denne target
    if route_manager.should_skip_current_target():
        print("🚫 TOO MANY ATTEMPTS ON THIS TARGET - SKIPPING")
        route_manager.advance_to_next_target()
        return
    
    direction = "right" if angle_diff > hitting_zone_max else "left"
    
    # Beregn hvor meget der skal drejes for at komme i hitting zone
    if angle_diff > hitting_zone_max:
        turn_amount = angle_diff - hitting_zone_max  # Drej til max hitting zone
    else:  # angle_diff < hitting_zone_min
        turn_amount = hitting_zone_min - angle_diff  # Drej til min hitting zone
    
    # KORRIGERET ROTATION: 180° = 0.5 rotations 
    rotations = turn_amount / 180.0 * 0.5
    
    # MINIMUM ROTATION: Rund små rotationer op til 0.01 for at sikre motor bevægelse
    min_rotation = 0.01
    if rotations < min_rotation:
        print("WARNING: Rotation {:.6f} for lille - rundet op til {:.3f}".format(rotations, min_rotation))
        rotations = min_rotation
    
    # BEGRÆNS ROTATION: Mindre rotationer for fin-justering
    # Store justeringer først, så fine justeringer
    if abs(angle_diff) > 15.0:
        max_rotations = 0.20  # Store korrektioner
    elif abs(angle_diff) > 5.0:
        max_rotations = 0.10  # Mellem korrektioner  
    else:
        max_rotations = 0.05  # Fine justeringer
        
    if rotations > max_rotations:
        rotations = max_rotations
        print("WARNING: Rotation begranset til {:.3f} for præcision (var {:.3f})".format(max_rotations, turn_amount / 180.0 * 0.5))
    
    print("ANGLE CORRECTION: {:.1f}deg -> hitting zone [{:.1f}, {:.1f}] -> {:.3f} rotations".format(
        angle_diff, hitting_zone_min, hitting_zone_max, rotations))
    commander.send_turn_rotation_command(direction, rotations)

def handle_forward_movement(distance_cm, angle_diff, commander):
    """Håndterer forsigtig fremadkørsel"""
    # Adaptive afstand baseret på nærhed til mål
    if distance_cm > 50:  # >10cm væk - normal hastighed
        move_distance = min(distance_cm - 25, 5)  # 3 cm steps til 22 cm
    elif distance_cm > 35:  # 7-10cm væk - langsommere
        move_distance = min(distance_cm - 25, 4)  # 2 cm steps til 22 cm
    else:  # <7cm væk - meget forsigtig
        move_distance = min(distance_cm - 25, 3)  # 1 cm steps til 22 cm
    
    print("IN HITTING ZONE - CAREFUL FORWARD {:.1f} cm (distance:{:.1f}cm, angle:{:.1f}deg) [Wall approach active]".format(
        move_distance, distance_cm, angle_diff))
    
    # Brug normal forward kommando (forward_precise findes ikke på robotten)
    commander.send_forward_command(move_distance)

def handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, 
                          in_hitting_zone, commander, route_manager):
    """Håndterer bold opsamling når robotten er tæt nok på"""
    print("=== READY FOR BLIND COLLECTION ===")
    print("Distance: {:.1f}cm ≤ 22cm, Angle: {:.1f}deg".format(distance_cm, angle_diff))
    print("Hitting zone: [{:.1f}, {:.1f}], In zone: {}".format(
        hitting_zone_min, hitting_zone_max, in_hitting_zone))
    print("Robot position good: distance ≤ 22cm AND in hitting zone")
    
    if in_hitting_zone:
        print("🎯 *** EXECUTING BLIND BALL COLLECTION *** 🎯")
        success = commander.send_blind_ball_collection_command()
        if success:
            print("✅ Blind collection command sent successfully!")
            # GÅ TIL NÆSTE PUNKT I RUTEN efter collection
            route_manager.advance_to_next_target()
        else:
            print("❌ Failed to send blind collection command!")
            # Øg antal forsøg og tjek om vi skal give op
            should_skip = route_manager.increment_collection_attempts()
            if should_skip:
                print("🚫 MAX ATTEMPTS REACHED - SKIPPING TO NEXT TARGET")
                route_manager.advance_to_next_target()
    else:
        print("❌ NOT IN HITTING ZONE - NEED ANGLE ADJUSTMENT FIRST")
        print("   Angle {:.1f}° is outside [{:.1f}°, {:.1f}°]".format(
            angle_diff, hitting_zone_min, hitting_zone_max))