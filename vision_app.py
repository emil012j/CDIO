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
    TOTAL_BALLS_ON_COURT = 12
    current_run_balls = 0
    total_balls_collected = 0
    previous_ball_count = 0

    global route_manager
    route_manager = RouteManager()

    last_print_time = time.time()
    frame_count = 0

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

            # Ball counting logic
            current_visible_balls = len(balls)
            if current_visible_balls < previous_ball_count:
                balls_difference = previous_ball_count - current_visible_balls
                current_run_balls += balls_difference
                total_balls_collected += balls_difference
                print("*** BALL(S) COLLECTED! Run: {}/{}, Total: {}/{} ***".format(
                    current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
            previous_ball_count = current_visible_balls

            navigation_info = None

            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )

                if current_state == COLLECTING:
                    if current_run_balls >= STORAGE_CAPACITY:
                        current_state = DELIVERING
                        print("*** STORAGE FULL - SWITCHING TO GOAL DELIVERY ***")
                        route_manager.reset_route()
                    elif not balls and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                        current_state = COMPLETE
                        print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                        route_manager.reset_route()
                    elif balls:
                        # Route-based navigation for balls
                        route_manager.create_route_from_balls(balls, robot_center, walls, cross_pos)
                        target_ball = route_manager.get_current_target()
                        if target_ball:
                            target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)
                            navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                            handle_robot_navigation(navigation_info, commander, route_manager)
                elif current_state == DELIVERING:
                    goal_position = goal_utils.get_goal_position()
                    if goal_position:
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, goal_position, scale_factor)
                        handle_robot_navigation(navigation_info, commander, route_manager)
                        # Check if close enough to goal to finish delivery
                        if navigation_info and navigation_info.get("distance_cm", 999) < 8:
                            print("*** REACHED GOAL - READY TO RELEASE ***")
                            current_run_balls = 0
                            if total_balls_collected >= TOTAL_BALLS_ON_COURT:
                                current_state = COMPLETE
                                print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                            else:
                                current_state = COLLECTING
                                print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN ***")
                            route_manager.reset_route()
                    else:
                        print("ERROR: No goal position set - cannot navigate to goal!")
                elif current_state == COMPLETE:
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
    