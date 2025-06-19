# -*- coding: utf-8 -*-
"""
Route visualization - håndterer tegning af rute, targets og robot navigation på skærmen
"""

import cv2
import math

def draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls, corrected_head=None, corrected_tail=None):
    """Tegn hele ruten og navigation information på skærmen"""
    if not robot_head or not robot_tail:
        return
    
    # Beregn robot centrum
    robot_center = (
        (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
        (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    )

    # Beregn korrigeret robot centrum, hvis tilgængeligt
    if corrected_head and corrected_tail:
        corrected_center = (
            (corrected_head[0] + corrected_tail[0]) // 2,
            (corrected_head[1] + corrected_tail[1]) // 2
        )
    else:
        corrected_center = robot_center # Brug standard centrum hvis korrigeret ikke findes

    # TEGN RUTE: Vis hele ruten og nuværende mål
    current_target = route_manager.get_current_target()
    if current_target:
        # Tjek om dette er en væg-tilgang (adjusted target)
        original_target = route_manager.route[route_manager.current_target_index] if route_manager.current_target_index < len(route_manager.route) else current_target
        is_wall_approach = (current_target != original_target)
        
        # Tegn linje til nuværende mål (GUL) fra det korrigerede/robot centrum
        cv2.line(display_frame, corrected_center, current_target, (255, 0, 255), 2)
        
        if is_wall_approach:
            # Tegn original bold position (CYAN)
            cv2.circle(display_frame, original_target, 8, (255, 255, 0), 2)
            cv2.putText(display_frame, "BALL", (original_target[0] + 10, original_target[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            # Tegn approach point (GUL)
            cv2.circle(display_frame, current_target, 15, (0, 255, 255), 3)
            cv2.putText(display_frame, "WALL APPROACH {}".format(route_manager.current_target_index + 1), 
                       (current_target[0] + 20, current_target[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            # Tegn linje fra approach til bold
            cv2.line(display_frame, current_target, original_target, (255, 255, 0), 2)
        else:
            # Normal bold target
            cv2.circle(display_frame, current_target, 15, (0, 255, 255), 2)
            cv2.putText(display_frame, "TARGET {}".format(route_manager.current_target_index + 1), 
                       (current_target[0] + 20, current_target[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Tegn hele ruten som farvede cirkler
    for i, waypoint in enumerate(route_manager.route):
        if i < route_manager.current_target_index:
            # Besøgte punkter: GRØN
            cv2.circle(display_frame, waypoint, 8, (0, 255, 0), -1)
            cv2.putText(display_frame, "✓", (waypoint[0] - 5, waypoint[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        elif i == route_manager.current_target_index:
            # Nuværende mål: allerede tegnet ovenfor
            pass
        else:
            # Fremtidige mål: BLÅ
            cv2.putText(display_frame, str(i + 1), (waypoint[0] - 5, waypoint[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    
    # TEGN VÆGGE (kun markering, ingen cirkler da vægge er aflange)
    for wall in walls:
        # Væg som lille rød firkant
        cv2.rectangle(display_frame, (wall[0]-10, wall[1]-10), (wall[0]+10, wall[1]+10), (0, 0, 255), -1)
        cv2.putText(display_frame, "WALL", (wall[0] + 15, wall[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

def draw_robot_heading(display_frame, robot_head, robot_tail):
    """Tegn robot retningslinje"""
    if not robot_head or not robot_tail:
        return
    
    # Tegn robot retning linje (tail→head extended)
    head_pos = robot_head["pos"] 
    tail_pos = robot_tail["pos"]
    dx = head_pos[0] - tail_pos[0]
    dy = head_pos[1] - tail_pos[1]
    if abs(dx) > 1e-6 or abs(dy) > 1e-6:
        length = math.sqrt(dx*dx + dy*dy)
        norm_dx = dx / length
        norm_dy = dy / length
        # Tegn linje fra tail gennem head og videre
        end_x = int(head_pos[0] + norm_dx * 200)
        end_y = int(head_pos[1] + norm_dy * 200)
        cv2.line(display_frame, tail_pos, (end_x, end_y), (255, 0, 255), 2)

def draw_route_status(display_frame, route_manager):
    """Tegn rute status information på skærmen"""
    if route_manager.route:
        route_text = "ROUTE: {}/{} waypoints (attempts: {}/{})".format(
            route_manager.current_target_index + 1, len(route_manager.route),
            route_manager.collection_attempts, route_manager.max_attempts)
        cv2.putText(display_frame, route_text, (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2) 