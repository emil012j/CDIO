# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
import math
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor, get_cross_position
from src.camera.coordinate_calculation import calculate_navigation_command, calculate_robot_heading, compute_approach_point
from src.camera.camera_manager import CameraManager, draw_navigation_info, display_status
from src.communication.vision_commander import VisionCommander
from src.navigation.route_manager import RouteManager
from src.navigation.navigation_logic import handle_robot_navigation
from src.visualization.route_visualization import draw_route_and_targets, draw_robot_heading, draw_route_status
from src.utils.vision_helpers import choose_unblocked_ball
from src.config.settings import *

# Global route manager
route_manager = RouteManager()
has_released = False
DROP_OFF_POINT = (150, 0)  # Will be dynamically set

def at_drop_off(current_pos, drop_off=DROP_OFF_POINT, threshold=15):
    dx = current_pos[0] - drop_off[0]
    dy = current_pos[1] - drop_off[1]
    return (dx**2 + dy**2)**0.5 < threshold

def select_top_wall(walls):
    return min(walls, key=lambda w: ((w[0][1] + w[1][1]) / 2)) if walls else None

def compute_wall_midpoint(wall):
    (x1, y1), (x2, y2) = wall
    return ((x1 + x2) / 2, (y1 + y2) / 2)

def main():
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

    print("Starting main loop -  escape to exit")

    last_print_time = time.time()
    frame_count = 0
    delivered_goal = False

    global DROP_OFF_POINT, has_released

    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break

            frame_count += 1
            current_time = time.time()

            results = run_detection(model, frame)
            scale_factor = calculate_scale_factor(results, model)
            cross_pos = get_cross_position(results, model)

            display_frame = frame.copy()
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(
                results, model, display_frame, scale_factor)

            navigation_info = None

            if robot_head and robot_tail and not balls:
                if route_manager.route_created:
                    route_manager.reset_route()

                if commander.can_send_command() and not delivered_goal:
                    print("*** ALL BALLS COLLECTED - PLANNING GOAL APPROACH ***")

                    robot_heading = calculate_robot_heading(robot_head, robot_tail)
                    if robot_heading is None:
                        print("[ERROR] Could not determine robot heading.")
                        continue

                    top_wall = select_top_wall(walls)
                    if top_wall is None:
                        print("[ERROR] No wall detected to use as goal.")
                        continue

                    DROP_OFF_POINT = compute_wall_midpoint(top_wall)
                    entry_point = compute_approach_point(DROP_OFF_POINT, 20, robot_heading)

                    print(f"[INFO] Goal at {DROP_OFF_POINT}, entry point: {entry_point}, heading: {robot_heading:.2f} deg")
                    route_manager.append_goal(entry_point)
                    delivered_goal = True

                # Trigger ball release if at drop-off
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                if at_drop_off(robot_center) and not has_released:
                    print("🏁 At drop-off point! Releasing balls...")
                    commander.send_release_ball_command()
                    has_released = True

            elif robot_head and robot_tail and balls:
                has_released = False
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                route_manager.create_route_from_balls(balls, robot_center, walls, cross_pos)
                target_ball = route_manager.get_current_target()

                if target_ball:
                    target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)

                if target_ball is None:
                    print("🎉 *** ROUTE COMPLETE - ALL TARGETS VISITED ***")
                    if commander.can_send_command():
                        commander.send_stop_command()
                else:
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)

                handle_robot_navigation(navigation_info, commander, route_manager)

            draw_route_and_targets(display_frame, robot_head, robot_tail, route_manager, walls)
            draw_robot_heading(display_frame, robot_head, robot_tail)

            if navigation_info:
                draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    balls[0] if balls else None,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"]
                )

            cv2.putText(display_frame, "Conf: {:.2f}".format(CONFIDENCE_THRESHOLD), (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            draw_route_status(display_frame, route_manager)
            cv2.imshow("YOLO OBB Detection", display_frame)

            if current_time - last_print_time >= PRINT_INTERVAL:
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Walls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO",
                    len(balls),
                    len(walls),
                    scale_factor if scale_factor else 0
                ))

                if not robot_head and not robot_tail:
                    print("*** KAN IKKE SE ROBOTTEN ***")
                elif not robot_head:
                    print("*** MANGLER ROBOT-HEAD ***")
                elif not robot_tail:
                    print("*** MANGLER ROBOT-TAIL ***")

                if not balls:
                    print("*** INGEN BOLDE FUNDET - MISSION COMPLETE? ***")
                else:
                    print("*** {} BOLDE SYNLIGE ***".format(len(balls)))

                print("\n--- Objekter koordinater @ {} ---".format(time.strftime('%H:%M:%S')))
                if log_info_list:
                    for info in log_info_list:
                        print(info)
                else:
                    print(" Ingen objekter detekteret over taersklen ({:.2f}).".format(CONFIDENCE_THRESHOLD))

                last_print_time = current_time

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
