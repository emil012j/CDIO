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
from src.navigation.navigation_logic import handle_robot_navigation, compute_path_around_cross
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
    STORAGE_CAPACITY = 6
    TOTAL_BALLS_ON_COURT = 7 # Important we write the correct number of balls we are testing with
    current_run_balls = 0
    total_balls_collected = 0
    previous_ball_count = 0

    global route_manager
    route_manager = RouteManager()

    last_print_time = time.time()
    frame_count = 0

    collected_balls_positions = []
    current_path_to_goal = [] # New: List to hold waypoints from compute_path_around_cross

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
            if cross_pos is None:
                scale_factor = 2.02 # Set fallback scale_factor if cross is not detected
            display_frame = frame.copy()
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)

            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                print("[DEBUG] robot_center:", robot_center)  # <--- DEBUG

            navigation_info = None

            if current_state == ROUTE_PLANNING:
                print("[DEBUG] State: ROUTE_PLANNING")
                # Removed filtering out balls inside the cross area based on CROSS_RADIUS.
                # The cross avoidance is now handled by compute_path_around_cross.
                # if balls and cross_pos:
                #     filtered_balls = []
                #     cross_center_x = cross_pos['center_x']
                #     cross_center_y = cross_pos['center_y']
                #     for ball in balls:
                #         ball_x, ball_y = ball
                #         dist_to_cross = ((ball_x - cross_center_x) ** 2 + (ball_y - cross_center_y) ** 2) ** 0.5
                #         if dist_to_cross > CROSS_RADIUS:
                #             filtered_balls.append(ball)
                #         else:
                #             print(f"[INFO] Skipping ball at {ball} because it is inside the cross area.")
                #     balls = filtered_balls
                # If we have balls, create a route and move to collection
                if balls and current_run_balls < STORAGE_CAPACITY:
                    # Pass the full cross_pos OBB dict to route_manager if it needs it for its route creation
                    # For now, it only needs a point, so passing the center for consistency if not refactoring route_manager
                    route_manager.create_route_from_balls(balls, robot_center, walls, (cross_pos['center_x'], cross_pos['center_y']) if cross_pos else None)
                    if route_manager.get_current_target():
                        current_state = BALL_COLLECTION
                        print("*** ROUTE PLANNED - SWITCHING TO BALL_COLLECTION ***")
                    else:
                        current_state = GOAL_NAVIGATION
                        current_path_to_goal = [] # Clear path if no balls found, and going to goal navigation
                        print("No valid target found in route planning, staying in ROUTE_PLANNING.")
                elif current_run_balls >= STORAGE_CAPACITY:
                    current_state = GOAL_NAVIGATION
                    current_path_to_goal = [] # Clear path when storage is full
                    print("*** STORAGE FULL - SWITCHING TO GOAL_NAVIGATION ***")
                    route_manager.reset_route()
                elif not balls and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                    print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls > 0: # Collected some, but no more visible balls
                    current_state = GOAL_NAVIGATION
                    current_path_to_goal = [] # Clear path when no more balls on field
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()
                else: # No balls found and not yet collected any, or all collected
                    commander.can_send_command()
                    commander.send_forward_command(distance=10)
                    current_state = GOAL_NAVIGATION
                    current_path_to_goal = [] # Clear path as a fallback
                    route_manager.reset_route()
                    print("No balls to collect or all collected, staying in ROUTE_PLANNING or COMPLETE.")

            elif current_state == BALL_COLLECTION:
                print("[DEBUG] State: BALL_COLLECTION")
                print("[DEBUG] current_run_balls:", current_run_balls, "balls:", balls)
                
                if current_run_balls >= STORAGE_CAPACITY:
                    current_state = GOAL_NAVIGATION
                    current_path_to_goal = [] # Clear path when storage is full
                    print("*** STORAGE FULL - SWITCHING TO GOAL_NAVIGATION ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls > 0: # Collected some, but no more visible balls
                    current_state = GOAL_NAVIGATION
                    current_path_to_goal = [] # Clear path when no more balls on field
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls == 0 and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                    print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    route_manager.reset_route()
                elif balls and current_run_balls < STORAGE_CAPACITY:
                    target_ball = route_manager.get_current_target()
                    if target_ball:
                        print(f"[DEBUG] BALL_COLLECTION: target_ball = {target_ball}")
                        print(f"[DEBUG] BALL_COLLECTION: current_path_to_goal (before check) = {current_path_to_goal}")

                        target_to_navigate = None
                        
                        # Condition to force recalculation of path
                        force_recalculate_path = False
                        if cross_pos and robot_head: # Cross is detected
                            if len(current_path_to_goal) == 1: # And current path is direct
                                current_immediate_target = current_path_to_goal[0]
                                if current_immediate_target == target_ball: # Check if the single waypoint is the ultimate target
                                    print("[DEBUG] BALL_COLLECTION: Cross detected, current path is direct to ball. Forcing recalculation.")
                                    force_recalculate_path = True

                        if not current_path_to_goal or force_recalculate_path:
                            # Calculate path if no waypoints are pending or forced recalculation
                            current_path_to_goal = [] # Clear before recalculating
                            if cross_pos and robot_head: # Ensure robot_head exists for robot_pos
                                print(f"[DEBUG] BALL_COLLECTION: Calling compute_path_around_cross with robot_head={{robot_head['pos']}}, target_ball={target_ball}, cross_pos={cross_pos}")
                                path_segment = compute_path_around_cross(robot_head["pos"], target_ball, cross_pos)
                                current_path_to_goal.extend(path_segment) # Add all waypoints/target to the list
                                print(f"[DEBUG] BALL_COLLECTION: compute_path_around_cross returned {path_segment}")
                                print(f"[DEBUG] BALL_COLLECTION: current_path_to_goal (after new path) = {current_path_to_goal}")
                            else:
                                # If no cross or robot_head, navigate directly to ball
                                print("[DEBUG] BALL_COLLECTION: No cross_pos or robot_head, navigating directly to ball.")
                                current_path_to_goal.append(target_ball)
                        
                        if current_path_to_goal:
                            target_to_navigate = current_path_to_goal[0] # Get the immediate waypoint/target
                            print(f"[DEBUG] BALL_COLLECTION: target_to_navigate = {target_to_navigate}")

                        if target_to_navigate:
                            # Use the immediate target for navigation calculation
                            navigation_info = calculate_navigation_command(robot_head, robot_tail, target_to_navigate, scale_factor)
                            collection_attempt_successful = handle_robot_navigation(navigation_info, commander, route_manager)
                            
                            # Check if the immediate target (waypoint or final ball) is reached
                            # This is a simplified check. A robust solution needs more precise 'waypoint reached' logic.
                            # For now, if navigation command was sent and distance to immediate target is small, assume reached.
                            if navigation_info and navigation_info.get("distance_cm", 999) < 40: # Increased threshold for waypoint reached
                                if len(current_path_to_goal) > 1: # If it was a waypoint
                                    print(f"Waypoint {current_path_to_goal[0]} reached. Moving to next segment.")
                                    current_path_to_goal.pop(0) # Remove the reached waypoint
                                else: # If it was the final ball target
                                    if collection_attempt_successful: # Only consider ball collected if handle_robot_navigation succeeded
                                        current_run_balls += 1
                                        total_balls_collected += 1
                                        print("*** BALL COLLECTED CONFIRMED! Run: {}/{}, Total: {}/{} ***".format(
                                            current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
                                        current_path_to_goal = [] # Clear path after collection
                                        route_manager.advance_to_next_target() # Move to next ball in route
                            else:
                                if navigation_info: # Only print distance if navigation_info is valid
                                    print(f"[DEBUG] BALL_COLLECTION: Distance to immediate target {navigation_info.get('distance_cm', 'N/A')} cm, not yet reached.")
                        else:
                            # This part might be reached if target_to_navigate somehow became None
                            print("DEBUG: No target to navigate to in BALL_COLLECTION.")
                            # If no target or path is determined, try replanning
                            current_state = ROUTE_PLANNING
                            print("*** NO TARGET OR PATH - REPLANNING ROUTE ***")
                    else:
                        # No more targets in current route, go back to route planning if more balls are on field
                        if balls:
                            current_state = ROUTE_PLANNING
                            print("*** ROUTE EXHAUSTED - REPLANNING ROUTE ***")
                        else: # No more balls to plan for
                            current_state = GOAL_NAVIGATION # Go to deliver what's collected
                            print("*** NO MORE BALLS TO PLAN FOR - DELIVERING WHAT'S COLLECTED ***")
                            route_manager.reset_route()
                            current_path_to_goal = [] # Clear any pending path
                else: # Fallback to route planning if no balls but not full storage and not collected all
                    current_state = ROUTE_PLANNING
                    print("No balls to collect, returning to ROUTE_PLANNING to re-evaluate.")

            elif current_state == GOAL_NAVIGATION:
                print("[DEBUG] State: GOAL_NAVIGATION") # <--- DEBUG
                goal_position = goal_utils.get_goal_position()
                print("[DEBUG] goal_position:", goal_position)  # <--- DEBUG
                if goal_position:
                    print(f"[DEBUG] GOAL_NAVIGATION: goal_position = {goal_position}")
                    print(f"[DEBUG] GOAL_NAVIGATION: current_path_to_goal (before check) = {current_path_to_goal}")

                    target_to_navigate = None

                    # Condition to force recalculation of path
                    force_recalculate_path = False
                    if cross_pos and robot_head: # Cross is detected
                        if len(current_path_to_goal) == 1: # And current path is direct
                            current_immediate_target = current_path_to_goal[0]
                            if current_immediate_target == goal_position: # Check if the single waypoint is the ultimate target
                                print("[DEBUG] GOAL_NAVIGATION: Cross detected, current path is direct to goal. Forcing recalculation.")
                                force_recalculate_path = True

                    if not current_path_to_goal or force_recalculate_path:
                        # Calculate path to goal if no waypoints are pending or forced recalculation
                        current_path_to_goal = [] # Clear before recalculating
                        if cross_pos and robot_head: # Ensure robot_head exists for robot_pos
                            print(f"[DEBUG] GOAL_NAVIGATION: Calling compute_path_around_cross with robot_head={{robot_head['pos']}}, goal_position={goal_position}, cross_pos={cross_pos}")
                            path_segment = compute_path_around_cross(robot_head["pos"], goal_position, cross_pos)
                            current_path_to_goal.extend(path_segment)
                            print(f"[DEBUG] GOAL_NAVIGATION: compute_path_around_cross returned {path_segment}")
                            print(f"[DEBUG] GOAL_NAVIGATION: current_path_to_goal (after new path) = {current_path_to_goal}")
                        else:
                            # If no cross or robot_head, navigate directly to goal
                            print("[DEBUG] GOAL_NAVIGATION: No cross_pos or robot_head, navigating directly to goal.")
                            current_path_to_goal.append(goal_position)
                    
                    if current_path_to_goal:
                        target_to_navigate = current_path_to_goal[0] # Get the immediate waypoint/target
                        print(f"[DEBUG] GOAL_NAVIGATION: target_to_navigate = {target_to_navigate}")

                    if target_to_navigate:
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, target_to_navigate, scale_factor)
                        print("Navigation info to goal:", navigation_info) # <--- DEBUG

                        current_distance_to_target = 999 # Default to a high value if navigation_info is None
                        if navigation_info:
                            current_distance_to_target = navigation_info.get("distance_cm", 999)
                        print(f"[DEBUG] Current distance to immediate target: {current_distance_to_target:.1f} cm") # New debug line

                        # Determine if robot is close enough to goal or needs to navigate
                        if current_distance_to_target < 22 and current_distance_to_target > 0: # Standardized threshold to 22cm
                            if len(current_path_to_goal) > 1: # If it was a waypoint
                                print(f"Waypoint {current_path_to_goal[0]} reached. Moving to next segment for GOAL_NAVIGATION.")
                                current_path_to_goal.pop(0) # Remove the reached waypoint
                            else: # If it was the final target in current_path_to_goal
                                if target_to_navigate == goal_position: # Check if the target is specifically the goal
                                    current_state = BALL_RELEASE
                                    print("*** REACHED GOAL - SWITCHING TO BALL_RELEASE ***")
                                else:
                                    # This case should ideally not be reached if logic is correct, but as a safeguard:
                                    print(f"DEBUG: Reached {target_to_navigate} but it's not the goal ({goal_position}). Staying in GOAL_NAVIGATION.")
                                    # You might want to add a fallback here, e.g., re-plan or advance route if this happens unexpectedly
                        elif navigation_info: # Only navigate if not yet at approach distance
                            print("[DEBUG] Calling handle_robot_navigation for goal (GOAL_NAVIGATION state)")  # <--- DEBUG
                            handle_robot_navigation(navigation_info, commander, route_manager)
                    else:
                        print("DEBUG: No target to navigate to in GOAL_NAVIGATION.")
                else:
                    print("ERROR: No goal position set - cannot navigate to goal!")

            elif current_state == BALL_RELEASE:
                print("[DEBUG] State: BALL_RELEASE")
                print("*** EXECUTING BALL RELEASE ***")
                commander.send_release_balls_command(duration=4) # Use 4 seconds as per last discussion
                current_run_balls = 0 # Reset collected balls for current run
                route_manager.reset_route() # Reset route after delivery
                current_path_to_goal = [] # Clear path after delivery

                if current_run_balls >= TOTAL_BALLS_ON_COURT: # Check if all balls are collected total
                    print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                else:
                    current_state = ROUTE_PLANNING # Loop back to route planning for next run
                    print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN (ROUTE_PLANNING) ***")
                
            # Visualization
            corrected_head_for_drawing = navigation_info.get('corrected_head') if navigation_info else None
            corrected_tail_for_drawing = navigation_info.get('corrected_tail') if navigation_info else None
            draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls, current_path_to_goal,
                                   corrected_head=corrected_head_for_drawing, 
                                   corrected_tail=corrected_tail_for_drawing)
            draw_robot_heading(display_frame, robot_head, robot_tail)
            draw_route_status(display_frame, route_manager)
            goal_utils.draw_goal_on_frame(display_frame)

            if navigation_info:
                # Use the same target that was used for navigation calculation
                # Now use the immediate target from current_path_to_goal for drawing
                actual_target = None
                if current_path_to_goal:
                    actual_target = current_path_to_goal[0]
                elif current_state == BALL_COLLECTION:
                    actual_target = route_manager.get_current_target() # Fallback, should be in current_path_to_goal
                elif current_state == GOAL_NAVIGATION:
                    actual_target = goal_utils.get_goal_position() # Fallback
                
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