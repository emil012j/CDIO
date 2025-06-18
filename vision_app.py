# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
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

    print("Loader YOLO model...")
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
    TOTAL_BALLS_ON_COURT = 1 # Vigtigt vi skriver det korrekte antal bolde vi tester med
    current_run_balls = 0
    total_balls_collected = 0
    previous_ball_count = 0

    global route_manager
    route_manager = RouteManager()

    last_print_time = time.time()
    frame_count = 0

    collected_balls_positions = []
    just_avoided_cross = False


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
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            


            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                print("[DEBUG] robot_center:", robot_center)  # <--- DEBUG

            navigation_info = None

             # --- Cross avoidance logic (runs in all states) ---
            if cross_pos and robot_head:
                head_x, head_y = robot_head["pos"]
                cross_x, cross_y = cross_pos
                dist_to_cross = ((head_x - cross_x) ** 2 + (head_y - cross_y) ** 2) ** 0.5
                if dist_to_cross <= 100 and not just_avoided_cross:  # Adjust threshold as needed
                    if cross_x > head_x:
                        turn_direction = "left"
                    else:
                        turn_direction = "right"
                    if commander.can_send_command():
                        print("*** CLOSE TO CROSS - GOING BACKWARDS AND TURNING ***")
                        commander.send_backward_command(distance=20)
                        time.sleep(1)
                        commander.send_turn_rotation_command(turn_direction, 0.5)
                        time.sleep(1)
                        commander.send_forward_command(distance=20)
                        time.sleep(1)
                        just_avoided_cross = True
                        continue  # Skip the rest of the loop to avoid further processing 
                    elif dist_to_cross < 120:
                        just_avoided_cross = False

            if current_state == ROUTE_PLANNING:
                print("[DEBUG] State: ROUTE_PLANNING")
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
                    #elif commander.can_send_command():
                     #   current_state = GOAL_NAVIGATION
                      #  route_manager.reset_route()
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
                        target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)
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
                    # Calculate navigation to goal
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, goal_position, scale_factor)
                    print("Navigation info to goal:", navigation_info) # <--- DEBUG

                    # Add detailed debug print for distance
                    current_distance_to_goal = 999 # Default to a high value if navigation_info is None
                    if navigation_info:
                        current_distance_to_goal = navigation_info.get("distance_cm", 999)
                    print(f"[DEBUG] Current distance to goal: {current_distance_to_goal:.1f} cm") # New debug line

                    # Determine if robot is close enough to goal or needs to navigate
                    if current_distance_to_goal < 26 and current_distance_to_goal > 0: # Use 22cm as threshold for goal approach as well
                        current_state = BALL_RELEASE
                        print("*** REACHED GOAL APPROACH DISTANCE - SWITCHING TO BALL_RELEASE ***")
                        # commander.send_release_balls_command(duration=4) # Moved to BALL_RELEASE state
                        # current_run_balls = 0 # Moved to BALL_RELEASE state
                        # if total_balls_collected >= TOTAL_BALLS_ON_COURT:
                        #     current_state = COMPLETE
                        #     print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                        # else:
                        #     current_state = ROUTE_PLANNING
                        #     print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN ***")
                        # route_manager.reset_route() # Moved to BALL_RELEASE state
                    elif navigation_info: # Only navigate if not yet at approach distance
                        print("[DEBUG] Calling handle_robot_navigation for goal (GOAL_NAVIGATION state)")  # <--- DEBUG
                        handle_robot_navigation(navigation_info, commander, route_manager)
                else:
                    print("ERROR: No goal position set - cannot navigate to goal!")

            elif current_state == BALL_RELEASE:
                print("[DEBUG] State: BALL_RELEASE")
                print("*** EXECUTING BALL RELEASE ***")
                commander.send_release_balls_command(duration=4) # Use 6 seconds as per last discussion
                commander.send_backward_command(distance=10)
                current_run_balls = 0 # Reset collected balls for current run
                route_manager.reset_route() # Reset route after delivery

                if current_run_balls >= TOTAL_BALLS_ON_COURT: # Check if all balls are collected total - havde total_balls_collected før i stedet for current_run_balls
                    
                    print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                else:
                    current_state = ROUTE_PLANNING # Loop back to route planning for next run
                    print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN (ROUTE_PLANNING) ***")
                
           

            # Visualization
            draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls)
            draw_robot_heading(display_frame, robot_head, robot_tail)
            draw_route_status(display_frame, route_manager)
            goal_utils.draw_goal_on_frame(display_frame)

            if navigation_info:
                target_pos = navigation_info.get('original_target')
                draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    target_pos,
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
        print("camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    
