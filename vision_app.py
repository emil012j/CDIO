# -*- coding: utf-8 -*-
"""
Main program that runs on the PC
Starts camera+YOLO, detects robot+balls, calculates navigation, sends commands, shows live video
"""

import cv2
import time
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor, get_cross_position
from src.camera.coordinate_calculation import calculate_navigation_command
from src.camera.camera_manager import CameraManager, draw_navigation_info, display_status
from src.communication.vision_commander import VisionCommander
from src.navigation.route_manager import RouteManager
from src.navigation.navigation_logic import handle_robot_navigation
from src.navigation.safe_spot_manager import SafeSpotManager
from src.visualization.route_visualization import draw_route_and_targets, draw_robot_heading, draw_route_status
from src.utils.vision_helpers import choose_unblocked_ball
from src.config.settings import *
from src.camera.goal_calibrator import GoalCalibrator
from src.camera.goal_utils import GoalUtils
# Global route manager
route_manager = RouteManager()

# --- Cross area radius in pixels (adjust as needed) ---
CROSS_RADIUS = 60

def main():
    print("Setting up goal position...")
    calibrator = GoalCalibrator()
    goal_utils = GoalUtils()
    goal_position = goal_utils.get_goal_position()
    if goal_position:
        print("Goal position loaded: {}".format(goal_position))
    else:
        print("No goal position found - calibration required!")
        print("You will need to calibrate the goal position before starting the mission.")
        print("Press 'c' during operation to start goal calibration.")

    print("Loading YOLO model...")
    model = load_yolo_model()
    if model is None:
        return

    print("Initializing camera...")
    camera = CameraManager()
    if not camera.initialize_camera():
        return

    print("Initializing vision commander...")
    commander = VisionCommander()

    # State management
    ROUTE_PLANNING = "route_planning"
    BALL_COLLECTION = "ball_collection"
    GOAL_NAVIGATION = "goal_navigation"
    BALL_RELEASE = "ball_release"

    current_state = ROUTE_PLANNING
    STORAGE_CAPACITY = 3
    TOTAL_BALLS_ON_COURT = 11 # Important we write the correct number of balls we are testing with
    current_run_balls = 0
    total_balls_collected = 0
    previous_ball_count = 0

    global route_manager
    route_manager = RouteManager()
    safe_spot_manager = SafeSpotManager()

    last_print_time = time.time()
    frame_count = 0

    collected_balls_positions = []
    just_avoided_cross = False
    cross_avoid_reset_time = None

    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break

            frame_count += 1
            current_time = time.time()

            # Detection
            results = run_detection(model, frame)
            scale_factor = calculate_scale_factor(results, model)
            cross_pos = get_cross_position(results, model)
            display_frame = frame.copy()
            robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)

            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                print("[DEBUG] robot_center:", robot_center)  # <--- DEBUG

            navigation_info = None

            # --- Cross avoidance flag reset (always runs) ---
            if just_avoided_cross:
                if cross_pos and robot_head:
                    head_x, head_y = robot_head["pos"]
                    cross_x, cross_y = cross_pos
                    dist_to_cross = ((head_x - cross_x) ** 2 + (head_y - cross_y) ** 2) ** 0.5
                    if dist_to_cross > 180:  # Use a larger threshold for reset
                        just_avoided_cross = False
                        cross_avoid_reset_time = None
                else:
                    if cross_avoid_reset_time is None:
                        cross_avoid_reset_time = time.time()
                    elif time.time() - cross_avoid_reset_time > 2:  # 2 seconds without seeing the cross
                        just_avoided_cross = False
                        cross_avoid_reset_time = None
            else:
                cross_avoid_reset_time = None

            # --- Cross avoidance logic (dynamic turn direction, using head position) ---
            if cross_pos and robot_head:
                head_x, head_y = robot_head["pos"]
                cross_x, cross_y = cross_pos
                dist_to_cross = ((head_x - cross_x) ** 2 + (head_y - cross_y) ** 2) ** 0.5
                if dist_to_cross <= 120 and not just_avoided_cross:
                    # Dynamic turn direction: turn away from cross
                    turn_direction = "right" if cross_x > head_x else "left"
                    if commander.can_send_command():
                        print(f"*** CLOSE TO CROSS - GOING BACKWARDS AND TURNING {turn_direction.upper()} ***")
                        commander.send_backward_command(distance=20)
                        time.sleep(1)
                        commander.send_turn_rotation_command(turn_direction, 0.5)
                        time.sleep(1)
                        commander.send_forward_command(distance=15)
                        time.sleep(1)
                        just_avoided_cross = True
                        continue

            if current_state == ROUTE_PLANNING:
                print("[DEBUG] State: ROUTE_PLANNING")
                # --- Filter out balls inside the cross area ---
                if balls and cross_pos:
                    filtered_balls = []
                    cross_x, cross_y = cross_pos
                    for ball in balls:
                        ball_x, ball_y = ball
                        dist_to_cross = ((ball_x - cross_x) ** 2 + (ball_y - cross_y) ** 2) ** 0.5
                        if dist_to_cross > CROSS_RADIUS:
                            filtered_balls.append(ball)
                        else:
                            print(f"[INFO] Skipping ball at {ball} because it is inside the cross area.")
                    balls = filtered_balls
                # If we have balls, create a route and move to collection
                if balls and current_run_balls < STORAGE_CAPACITY:
                    route_manager.create_route_from_balls(balls, robot_center, cross_pos)
                    if route_manager.get_current_target():
                        current_state = BALL_COLLECTION
                        print("*** ROUTE PLANNED - SWITCHING TO BALL_COLLECTION ***")
                    else:
                        current_state = GOAL_NAVIGATION
                        print("No valid target found in route planning, staying in ROUTE_PLANNING.")
                elif current_run_balls >= STORAGE_CAPACITY:
                    current_state = GOAL_NAVIGATION
                    print("*** STORAGE FULL - SWITCHING TO GOAL_NAVIGATION ***")
                    route_manager.reset_route()
                elif not balls and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                    print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls > 0: # Collected some, but no more visible balls
                    current_state = GOAL_NAVIGATION
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()
                else: # No balls found and not yet collected any, or all collected
                    commander.can_send_command()
                    commander.send_forward_command(distance=10)
                    current_state = GOAL_NAVIGATION
                    route_manager.reset_route()
                    print("No balls to collect or all collected, staying in ROUTE_PLANNING or COMPLETE.")

            elif current_state == BALL_COLLECTION:
                print("[DEBUG] State: BALL_COLLECTION")
                print("[DEBUG] current_run_balls:", current_run_balls, "balls:", balls)
                
                if current_run_balls >= STORAGE_CAPACITY:
                    current_state = GOAL_NAVIGATION
                    print("*** STORAGE FULL - SWITCHING TO GOAL_NAVIGATION ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls > 0: # Collected some, but no more visible balls
                    current_state = GOAL_NAVIGATION
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls == 0 and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                    print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    route_manager.reset_route()
                elif balls and current_run_balls < STORAGE_CAPACITY:
                    target_ball = route_manager.get_current_target()
                    if target_ball:
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                        # Capture success of navigation/collection attempt
                        collection_attempt_successful = handle_robot_navigation(navigation_info, commander, route_manager)
                        if collection_attempt_successful:
                            current_run_balls += 1
                            total_balls_collected += 1
                            print("*** BALL COLLECTED CONFIRMED! Run: {}/{}, Total: {}/{} ***".format(
                                current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
                    else:
                        # No more targets in current route, go back to route planning if more balls are on field
                        if balls:
                            current_state = ROUTE_PLANNING
                            print("*** ROUTE EXHAUSTED - REPLANNING ROUTE ***")
                        else: # No more balls to plan for
                            current_state = GOAL_NAVIGATION # Go to deliver what's collected
                            print("*** NO MORE BALLS TO PLAN FOR - DELIVERING WHAT'S COLLECTED ***")
                            route_manager.reset_route()
                else: # Fallback to route planning if no balls but not full storage and not collected all
                    current_state = ROUTE_PLANNING
                    print("No balls to collect, returning to ROUTE_PLANNING to re-evaluate.")

            elif current_state == GOAL_NAVIGATION:
                print("[DEBUG] State: GOAL_NAVIGATION") # <--- DEBUG
                goal_position = goal_utils.get_goal_position()
                print("[DEBUG] goal_position:", goal_position)  # <--- DEBUG
                if goal_position:
                    # Create safe route to goal if not already created
                    if not route_manager.route_created:
                        goal_marker = goal_utils.get_goal_marker()
                        route_manager.create_goal_route(robot_center, goal_position, goal_marker)
                    
                    target_position = route_manager.get_current_target()
                    if target_position:
                        # Calculate navigation to current target (waypoint or goal)
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, target_position, scale_factor)
                        print("Navigation info to target:", navigation_info) # <--- DEBUG

                        # Add detailed debug print for distance
                        current_distance_to_target = 999 # Default to a high value if navigation_info is None
                        if navigation_info:
                            current_distance_to_target = navigation_info.get("distance_cm", 999)
                        print(f"[DEBUG] Current distance to target: {current_distance_to_target:.1f} cm") # New debug line

                        # Check if we reached current target
                        if current_distance_to_target < 26 and current_distance_to_target > 0:
                            if route_manager.is_current_target_waypoint():
                                print("*** REACHED WAYPOINT - ADVANCING TO NEXT TARGET ***")
                                route_manager.advance_to_next_target()
                            elif target_position == goal_position: # Check if current target is the final goal
                                # Reached final goal
                                current_state = BALL_RELEASE
                                print("*** REACHED GOAL - SWITCHING TO BALL_RELEASE ***")
                            else: # We are at an intermediate point but not a waypoint. This should not happen if route planning is correct.
                                print("WARNING: Reached intermediate point not classified as waypoint. Advancing to next target.")
                                route_manager.advance_to_next_target()
                        elif navigation_info: # Only navigate if not yet at target
                            print("[DEBUG] Calling handle_robot_navigation for target")  # <--- DEBUG
                            handle_robot_navigation(navigation_info, commander, route_manager)
                    else:
                        print("ERROR: No target in goal route!")
                else:
                    print("ERROR: No goal position set - cannot navigate to goal!")

            elif current_state == BALL_RELEASE:
                print("[DEBUG] State: BALL_RELEASE")
                print("*** EXECUTING BALL RELEASE ***")
                commander.send_release_balls_command(duration=4) # Use 4 seconds as per last discussion
                current_run_balls = 0 # Reset collected balls for current run
                route_manager.reset_route() # Reset route after delivery

                if current_run_balls >= TOTAL_BALLS_ON_COURT: # Check if all balls are collected total
                    print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                else:
                    current_state = ROUTE_PLANNING # Loop back to route planning for next run
                    print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN (ROUTE_PLANNING) ***")
                
            # Visualization
            corrected_head_for_drawing = navigation_info.get('corrected_head') if navigation_info else None
            corrected_tail_for_drawing = navigation_info.get('corrected_tail') if navigation_info else None
            draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, 
                                   corrected_head=corrected_head_for_drawing, 
                                   corrected_tail=corrected_tail_for_drawing)
            draw_robot_heading(display_frame, robot_head, robot_tail)
            draw_route_status(display_frame, route_manager)
            goal_utils.draw_goal_on_frame(display_frame)
            goal_utils.draw_marker_on_frame(display_frame)
            
            # Draw safe spots and quadrant system
            safe_spot_manager.draw_safe_spots(display_frame)

            if navigation_info:
                # Use the same target that was used for navigation calculation
                if current_state == BALL_COLLECTION:
                    actual_target = route_manager.get_current_target()  # Same target used for navigation
                elif current_state == GOAL_NAVIGATION:
                    actual_target = goal_utils.get_goal_position()
                else:
                    actual_target = None
                    
                draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    actual_target,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"],
                    navigation_info
                )

            display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            cv2.imshow("YOLO OBB Detection", display_frame)

            # Keyboard controls
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                print("*** STARTING GOAL CALIBRATION ***")
                calibrator.start_calibration(frame)
                goal_utils.load_goal()
                goal_position = goal_utils.get_goal_position()
                if goal_position:
                    print("Goal calibration complete: {}".format(goal_position))
                else:
                    print("Goal calibration was cancelled or failed")

            # Status print (unchanged)
            if current_time - last_print_time >= PRINT_INTERVAL:
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls),
                    scale_factor if scale_factor else 0
                ))
                last_print_time = current_time

    except KeyboardInterrupt:
        print("Camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()