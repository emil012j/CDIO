# -*- coding: utf-8 -*-
"""
Route visualization - handles drawing of route, targets, and robot navigation on the screen
"""

import cv2
import math

def draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls, corrected_head=None, corrected_tail=None):
    """Draw the entire route and navigation information on the screen"""
    if not robot_head or not robot_tail:
        return
    
    # Calculate robot center
    robot_center = (
        (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
        (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    )

    # Calculate corrected robot center, if available
    if corrected_head and corrected_tail:
        corrected_center = (
            (corrected_head[0] + corrected_tail[0]) // 2,
            (corrected_head[1] + corrected_tail[1]) // 2
        )
    else:
        corrected_center = robot_center # Use default center if corrected not available

    # DRAW ROUTE: Show the entire route and current target
    current_target = route_manager.get_current_target()
    if current_target:
        # Check if this is a wall approach (adjusted target)
        original_target = route_manager.route[route_manager.current_target_index] if route_manager.current_target_index < len(route_manager.route) else current_target
        is_wall_approach = (current_target != original_target)
        
        # Draw line to current target (YELLOW) from corrected/robot center
        cv2.line(display_frame, corrected_center, current_target, (255, 0, 255), 2)
        
        if is_wall_approach:
            # Draw original ball position (CYAN)
            cv2.circle(display_frame, original_target, 8, (255, 255, 0), 2)
            cv2.putText(display_frame, "BALL", (original_target[0] + 10, original_target[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            # Draw approach point (YELLOW)
            cv2.circle(display_frame, current_target, 15, (0, 255, 255), 3)
            cv2.putText(display_frame, "WALL APPROACH {}".format(route_manager.current_target_index + 1), 
                       (current_target[0] + 20, current_target[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            # Draw line from approach to ball
            cv2.line(display_frame, current_target, original_target, (255, 255, 0), 2)
        else:
            # Normal ball target
            cv2.circle(display_frame, current_target, 15, (0, 255, 255), 2)
            cv2.putText(display_frame, "TARGET {}".format(route_manager.current_target_index + 1), 
                       (current_target[0] + 20, current_target[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Draw the entire route as colored circles
    for i, waypoint in enumerate(route_manager.route):
        if i < route_manager.current_target_index:
            # Visited points: GREEN
            cv2.circle(display_frame, waypoint, 8, (0, 255, 0), -1)
            cv2.putText(display_frame, "✓", (waypoint[0] - 5, waypoint[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        elif i == route_manager.current_target_index:
            # Current target: already drawn above
            pass
        else:
            # Future targets: BLUE
            cv2.putText(display_frame, str(i + 1), (waypoint[0] - 5, waypoint[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    
    # DRAW WALLS (just marking, no circles since walls are elongated)
    for wall in walls:
        # Wall as small red square
        cv2.rectangle(display_frame, (wall[0]-10, wall[1]-10), (wall[0]+10, wall[1]+10), (0, 0, 255), -1)
        cv2.putText(display_frame, "WALL", (wall[0] + 15, wall[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

def draw_robot_heading(display_frame, robot_head, robot_tail):
    """Draw robot heading line"""
    if not robot_head or not robot_tail:
        return
    
    # Draw robot heading line (tail→head extended)
    head_pos = robot_head["pos"] 
    tail_pos = robot_tail["pos"]
    dx = head_pos[0] - tail_pos[0]
    dy = head_pos[1] - tail_pos[1]
    if abs(dx) > 1e-6 or abs(dy) > 1e-6:
        length = math.sqrt(dx*dx + dy*dy)
        norm_dx = dx / length
        norm_dy = dy / length
        # Draw line from tail through head and further
        end_x = int(head_pos[0] + norm_dx * 200)
        end_y = int(head_pos[1] + norm_dy * 200)
        cv2.line(display_frame, tail_pos, (end_x, end_y), (255, 0, 255), 2)

def draw_route_status(display_frame, route_manager):
    """Draw route status information on the screen"""
    if route_manager.route:
        # Calculate elapsed time on current target
        elapsed_time = 0.0
        if route_manager.target_start_time:
            import time
            elapsed_time = time.time() - route_manager.target_start_time
        
        route_text = "ROUTE: {}/{} waypoints (attempts: {}/{}, time: {:.1f}s/{:.0f}s)".format(
            route_manager.current_target_index + 1, len(route_manager.route),
            route_manager.collection_attempts, route_manager.max_attempts,
            elapsed_time, route_manager.max_target_time)
        cv2.putText(display_frame, route_text, (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2) 