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
    #PUSH_CROSS = "push_cross"
    ROUTE_PLANNING = "route_planning"
    BALL_COLLECTION = "ball_collection"
    GOAL_NAVIGATION = "goal_navigation"
    BALL_RELEASE = "ball_release"

    current_state = ROUTE_PLANNING # Initial state
    #push_cross_start_time = 0
    #push_cross_duration = 10 # 10 seconds
    #pushed_cross_once = False # Flag to ensure PUSH_CROSS runs only once

    STORAGE_CAPACITY = 6
    TOTAL_BALLS_ON_COURT = 11 # Important we write the correct number of balls we are testing with
    current_run_balls = 0
    total_balls_collected = 0
    previous_ball_count = 0

    global route_manager
    route_manager = RouteManager()

    last_print_time = time.time()
    frame_count = 0

    collected_balls_positions = []

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
            # If cross is not found, set scale_factor to 2.02
            if scale_factor is None:
                scale_factor = 2.02
            display_frame = frame.copy()
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)

            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )

            navigation_info = None

            if current_state == ROUTE_PLANNING:
                print("[DEBUG] State: ROUTE_PLANNING")
                # --- Filter out balls inside the cross area only if cross is found ---
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
                # If cross_pos is None, do not filter balls
                # If we have balls, create a route and move to collection
                if balls and current_run_balls < STORAGE_CAPACITY:
                    route_manager.create_route_from_balls(balls, robot_center, walls, cross_pos)
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
                        # Use the actual ball position directly - no wall approach modification
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
                if goal_position:
                    # Calculate navigation to goal
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, goal_position, scale_factor)

                    # Add detailed debug print for distance
                    current_distance_to_goal = 999 # Default to a high value if navigation_info is None
                    if navigation_info:
                        current_distance_to_goal = navigation_info.get("distance_cm", 999)

                    # Determine if robot is close enough to goal or needs to navigate
                    if current_distance_to_goal < 26 and current_distance_to_goal > 0: # Use 22cm as threshold for goal approach as well
                        current_state = BALL_RELEASE
                        print("*** REACHED GOAL APPROACH DISTANCE - SWITCHING TO BALL_RELEASE ***")
                    elif navigation_info:
                        # The handle_robot_navigation function already checks if a command can be sent
                        handle_robot_navigation(navigation_info, commander, route_manager)
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
            draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls, 
                                   corrected_head=corrected_head_for_drawing, 
                                   corrected_tail=corrected_tail_for_drawing)
            draw_robot_heading(display_frame, robot_head, robot_tail)
            draw_route_status(display_frame, route_manager)
            goal_utils.draw_goal_on_frame(display_frame)

            # --- TIMER VISUALIZATION ---
            # Show timer for current target if route is active and in BALL_COLLECTION
            if current_state == BALL_COLLECTION and route_manager.route and not route_manager.is_route_complete():
                if route_manager.target_start_time is not None:
                    elapsed = time.time() - route_manager.target_start_time
                    remaining = max(0, int(route_manager.max_target_time - elapsed))
                    timer_text = f"Target-timer: {remaining}s tilbage"
                    cv2.putText(display_frame, timer_text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
                    # If timeout is reached, show a warning
                    if remaining == 0:
                        cv2.putText(display_frame, "SPRINGER OVER: Timeout!", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)

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
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Walls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls),
                    len(walls),
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