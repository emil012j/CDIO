# -*- coding: utf-8 -*-
"""
Navigation logic for the robot - handles turning and forward commands
"""

import time
import math

# --- Helper Geometric Functions ---

def dist_points(p1, p2):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def orientation(p, q, r):
    """
    Finds the orientation of ordered triplet (p, q, r).
    0 --> p, q and r are collinear
    1 --> Clockwise
    2 --> Counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - \
          (q[0] - p[0]) * (r[1] - q[1])
    if val == 0: return 0  # Collinear
    return 1 if val > 0 else 2 # Clockwise or Counterclockwise

def on_segment(p, q, r):
    """
    Checks if point q lies on line segment 'pr'.
    """
    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

def intersect_segments(p1, q1, p2, q2):
    """
    Checks if line segment 'p1q1' and 'p2q2' intersect.
    """
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != 0 and o2 != 0 and o3 != 0 and o4 != 0 and \
       o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if o1 == 0 and on_segment(p1, p2, q1): return True
    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if o2 == 0 and on_segment(p1, q2, q1): return True
    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if o3 == 0 and on_segment(p2, p1, q2): return True
    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if o4 == 0 and on_segment(p2, q1, q2): return True

    return False # Doesn't fall in any of the above cases

def rotate_point(point, origin, angle_degrees):
    """
    Rotates a point around an origin by a given angle in degrees.
    """
    angle_rad = math.radians(angle_degrees)
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle_rad) * (px - ox) - math.sin(angle_rad) * (py - oy)
    qy = oy + math.sin(angle_rad) * (px - ox) + math.cos(angle_rad) * (py - oy)
    return (qx, qy)

def get_obb_corners(center_x, center_y, width, height, angle_degrees):
    """
    Calculates the four corner points of an Oriented Bounding Box (OBB).
    Angle is in degrees, typically from YOLO, and represents rotation around the Z-axis.
    YOLO angles often follow a convention where 0 degrees is horizontal,
    and positive angles rotate counter-clockwise.
    We assume the angle is relative to the "upright" bounding box.
    """
    half_width = width / 2
    half_height = height / 2

    # Unrotated corners relative to center
    unrotated_corners = [
        (-half_width, -half_height),  # Bottom-left
        (half_width, -half_height),   # Bottom-right
        (half_width, half_height),    # Top-right
        (-half_width, half_height)    # Top-left
    ]

    # Rotate each corner around the OBB center
    rotated_corners = []
    for corner in unrotated_corners:
        rotated_corners.append(rotate_point(corner, (0,0), angle_degrees))
    
    # Translate corners to world coordinates
    return [(cx + center_x, cy + center_y) for cx, cy in rotated_corners]

# --- Main Pathfinding Logic ---

def compute_path_around_cross(robot_pos, goal_pos, cross_rect):
    """
    Computes a path from robot_pos to goal_pos, navigating around a cross
    defined by cross_rect if a direct path intersects it.

    Args:
        robot_pos (tuple): (x, y) coordinates of the robot's head.
        goal_pos (tuple): (x, y) coordinates of the target goal (ball).
        cross_rect (dict): Dictionary defining the cross's OBB,
                           e.g., {'center_x': X, 'center_y': Y, 'width': W, 'height': H, 'angle': A}.

    Returns:
        list: A list of (x, y) waypoints for the robot to navigate to in order.
              Returns [goal_pos] if no intersection or direct path is clear.
              Returns [waypoint, goal_pos] if an alternative path is needed.
    """
    cross_corners = get_obb_corners(
        cross_rect['center_x'], cross_rect['center_y'],
        cross_rect['width'], cross_rect['height'], cross_rect['angle']
    )

    # Define the line segment for the direct path
    robot_segment = (robot_pos, goal_pos)

    # Check for intersection with any of the four edges of the cross OBB
    intersects = False
    for i in range(4):
        cross_edge = (cross_corners[i], cross_corners[(i + 1) % 4])
        if intersect_segments(robot_segment[0], robot_segment[1],
                              cross_edge[0], cross_edge[1]):
            intersects = True
            break
    
    if not intersects:
        print("Direct path is clear, no intersection with cross.")
        return [goal_pos]
    
    # If intersects, calculate an alternative waypoint
    print("Direct path intersects cross. Calculating alternative route.")
    
    cross_center = (cross_rect['center_x'], cross_rect['center_y'])

    # Determine a perpendicular direction to move away from the direct path, relative to the cross's center
    # This simplified approach aims to push the waypoint away from the direct path.
    # A more robust solution might consider the orientation of the cross and the approach angle.
    
    # Vector from robot to goal
    vx = goal_pos[0] - robot_pos[0]
    vy = goal_pos[1] - robot_pos[1]

    # Vector from robot to cross center
    cx = cross_center[0] - robot_pos[0]
    cy = cross_center[1] - robot_pos[1]

    # Calculate the cross product to determine on which side the cross center lies
    # (relative to the vector from robot_pos to goal_pos)
    cross_product = vx * cy - vy * cx

    # Offset distance for the waypoint - make it a bit larger than the cross's half-width/height
    offset_distance = max(cross_rect['width'], cross_rect['height']) / 2 + 200 # Increased offset for more buffer

    # Calculate a vector perpendicular to (vx, vy)
    # If cross_product > 0, cross is to the "left" (counter-clockwise)
    # If cross_product < 0, cross is to the "right" (clockwise)
    if cross_product > 0:
        # Move waypoint to the "right" (clockwise perpendicular)
        perp_vx = vy
        perp_vy = -vx
    else:
        # Move waypoint to the "left" (counter-clockwise perpendicular)
        perp_vx = -vy
        perp_vy = vx
    
    # Normalize the perpendicular vector
    mag_perp = math.sqrt(perp_vx**2 + perp_vy**2)
    if mag_perp == 0: # Should not happen unless robot_pos == goal_pos
        print("Robot position is same as goal position. Cannot calculate offset.")
        return [goal_pos]
        
    perp_vx /= mag_perp
    perp_vy /= mag_perp

    # Calculate the waypoint by offsetting the cross's center
    # This waypoint is offset from the cross center, in a direction perpendicular to the robot-goal line
    waypoint_x = cross_center[0] + perp_vx * offset_distance
    waypoint_y = cross_center[1] + perp_vy * offset_distance
    
    waypoint = (waypoint_x, waypoint_y)
    print(f"Calculated waypoint: {waypoint}")
    return [waypoint, goal_pos]

def handle_robot_navigation(navigation_info, commander, route_manager):
    """Main navigation logic - handles turning and forward movement"""
    if not navigation_info or not commander.can_send_command():
        return
    
    angle_diff = navigation_info["angle_diff"]
    distance_cm = navigation_info["distance_cm"]
    
    print("Navigation: Target={} Angle diff={:.1f}deg, Distance={:.1f}cm".format(
        "Available", angle_diff, distance_cm))
    
    # PRECISE HITTING ZONE: Tighter zone for better precision
    hitting_zone_min = -1.0  # Tighter: Minimum precise angle to hit the ball
    hitting_zone_max = 1.0   # Tighter: Maximum precise angle to hit the ball
    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
    
    # DEBUG: Show current state vs thresholds every frame
    print("  DEBUG: Distance={:.1f}cm (<=22cm?), Angle={:.1f}° in [{:.1f}°,{:.1f}°]? = {}".format(
        distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone))
    
    # TURN PHASE: Correct angle if not in hitting zone
    if not in_hitting_zone:
        # handle_turn_correction now handles skipping, so we don't return from here
        handle_turn_correction(angle_diff, hitting_zone_min, hitting_zone_max, commander, route_manager)
        return False # No ball collected during turn
    
    # FORWARD PHASE: Careful forward movement when in hitting zone
    elif distance_cm > 22:  # Stop when we are 22 cm away for blind collection
        handle_forward_movement(distance_cm, angle_diff, commander)
        return False # No ball collected during forward movement
    
    # BLIND BALL COLLECTION: In hitting zone AND \u226422 cm away - start blind collection
    else:
        return handle_ball_collection(distance_cm, angle_diff, -1.0, 1.0, # Precise hitting zone for collection
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

def handle_ball_collection(distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, 
                          in_hitting_zone, commander, route_manager):
    """Handles ball collection when the robot is close enough"""
    print("=== READY FOR BLIND COLLECTION ===")
    print("Distance: {:.1f}cm <= 22cm, Angle: {:.1f}deg".format(distance_cm, angle_diff))
    print("Hitting zone: [{:.1f}, {:.1f}], In zone: {}".format(
        hitting_zone_min, hitting_zone_max, in_hitting_zone))
    print("Robot position good: distance <= 22cm AND in hitting zone")
    
    # Increment collection attempts (if not already maximum attempts)
    # This should happen before sending the command, so if the command fails, we still count the attempt.
    if not route_manager.should_skip_current_target():
        route_manager.increment_collection_attempts()

    # Send the blind collection command
    print("*** EXECUTING BLIND BALL COLLECTION ***")
    # Assuming send_blind_collection_command returns True on success, False on failure.
    # If it does not return anything, we will assume it is always successful for now.
    collection_successful = commander.send_blind_collection_command(duration=2) # Default 2 seconds
    
    if collection_successful:
        print("Blind collection command sent successfully!")
        return True
    else:
        print("Blind collection command FAILED!")
        return False

# Helper function for getting precise navigation values
def get_precise_navigation_values(navigation_info):
    # Implementation of get_precise_navigation_values function
    pass