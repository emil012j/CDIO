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
    COLLECTING = "collecting"
    DELIVERING = "delivering"
    COMPLETE = "complete"
    current_state = COLLECTING
    STORAGE_CAPACITY = 6
    TOTAL_BALLS_ON_COURT = 7 # Vigtigt vi skriver det korrekte antal bolde vi tester med
    current_run_balls = 0
    total_balls_collected = 0

    global route_manager
    route_manager = RouteManager()

    last_print_time = time.time()
    frame_count = 0

    
    ball_proximity_counter = {}
    collected_balls_positions = []

    def is_ball_collected(ball_pos, robot_center, radius=50):
        dx = ball_pos[0] - robot_center[0]
        dy = ball_pos[1] - robot_center[1]
        return (dx*dx + dy*dy) < (radius*radius)

    COLLECTION_RADIUS = 30      # pixels - hvor tæt robotten skal være på bolden for at tælle som indsamlet
    COLLECTION_FRAMES = 8       # frames robot must be close to count as collected - ( 8 frames = 0.2 seconds at 40 FPS )

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
                print("[DEBUG] robot_center:", robot_center)

                # --- Proximity+time ball collection logic ---
                if 'target_ball' in locals() and target_ball:
                    if current_run_balls >= STORAGE_CAPACITY:
                        print("[DEBUG] Storage full - skipping collection tracking")
                        target_ball = None
                    else:
                        if is_ball_collected(target_ball, robot_center, radius=COLLECTION_RADIUS):
                            ball_proximity_counter[target_ball] = ball_proximity_counter.get(target_ball, 0) + 1
                        else:
                            ball_proximity_counter[target_ball] = 0

                    if ball_proximity_counter.get(target_ball, 0) >= COLLECTION_FRAMES and target_ball not in collected_balls_positions:
                        current_run_balls += 1
                        total_balls_collected += 1
                        collected_balls_positions.append(target_ball)
                        print("*** BALL COLLECTED NEAR ROBOT (proximity+time)! Run: {}/{}, Total: {}/{} ***".format(
                            current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
                        print(f"[DEBUG] current_run_balls after collection: {current_run_balls}")
                        del ball_proximity_counter[target_ball]
                        route_manager.advance_to_next_target()  # Move to next ball in the route

                        if current_run_balls >= STORAGE_CAPACITY:
                            print("[DEBUG] Storage capacity reached. Skipping further collection.")
                            target_ball = None
                            continue

                navigation_info = None

                if current_state == COLLECTING:
                    print("[DEBUG] State: COLLECTING")
                    print("[DEBUG] current_run_balls:", current_run_balls, "balls:", balls)

                    # 1. STORAGE FULL? Go deliver!
                    if current_run_balls >= STORAGE_CAPACITY:
                        current_state = DELIVERING
                        print("*** STORAGE FULL - SWITCHING TO GOAL DELIVERY ***")
                        route_manager.reset_route()
                        continue  # or continue, depending on your loop structure

                    # 2. NO BALLS LEFT, STILL CARRYING? Go deliver!
                    if not balls and current_run_balls > 0:
                        current_state = DELIVERING
                        print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                        route_manager.reset_route()
                        continue

                    # 3. NO BALLS LEFT, NOTHING CARRIED, ALL COLLECTED? Mission complete!
                    if not balls and current_run_balls == 0 and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                        current_state = COMPLETE
                        print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                        route_manager.reset_route()
                        continue

                    # 4. Otherwise, keep collecting
                    if balls and current_run_balls < STORAGE_CAPACITY:
                        # Route-based navigation for balls
                        remaining_balls = [b for b in balls if b not in collected_balls_positions]
                        print("[DEBUG] remaining_balls:", remaining_balls)
                        if current_run_balls >= STORAGE_CAPACITY:
                            print("[DEBUG] Storage full, skipping route creation")
                            continue

                        if not route_manager.route_created or route_manager.is_route_complete():
                            print("[DEBUG] Creating new route from remaining balls...")
                            route_manager.reset_route()
                            route_manager.create_route_from_balls(remaining_balls, robot_center, walls, cross_pos)

                        target_ball = route_manager.get_current_target()

                        # 🔒 Prevent setting a target if storage is already full
                        if current_run_balls >= STORAGE_CAPACITY:
                            print("[DEBUG] Storage full, skipping target assignment")
                            continue

                        print("[DEBUG] target_ball:", target_ball)
                        if target_ball:
                            target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)
                            navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                            if navigation_info:
                                handle_robot_navigation(navigation_info, commander, route_manager)

                elif current_state == DELIVERING:
                    print("[DEBUG] State: DELIVERING")
                    goal_position = goal_utils.get_goal_position()
                    print("[DEBUG] goal_position:", goal_position)
                    if goal_position:
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, goal_position, scale_factor)
                        print("Navigation info to goal:", navigation_info)
                        current_distance_to_goal = 999
                        if navigation_info:
                            current_distance_to_goal = navigation_info.get("distance_cm", 999)
                        print(f"[DEBUG] Current distance to goal: {current_distance_to_goal:.1f} cm")
                        if current_distance_to_goal < 22 and current_distance_to_goal > 0:
                            print("*** REACHED GOAL APPROACH DISTANCE - READY TO RELEASE ***")
                            commander.send_release_balls_command(duration=4)
                            current_run_balls = 0
                            if total_balls_collected >= TOTAL_BALLS_ON_COURT and len(balls) == 0:
                                current_state = COMPLETE
                                print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                            else:
                                current_state = COLLECTING
                                print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN ***")
                            route_manager.reset_route()
                        elif navigation_info:
                            print("[DEBUG] Calling handle_robot_navigation for goal (DELIVERING state)")
                            handle_robot_navigation(navigation_info, commander, route_manager)
                    else:
                        print("ERROR: No goal position set - cannot navigate to goal!")

                elif current_state == COMPLETE:
                    print("[DEBUG] State: COMPLETE")
                    if commander.can_send_command():
                        print("*** MISSION COMPLETE - STOPPING ROBOT ***")
                        commander.send_stop_command()

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
    
