# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
from src.camera.detection import load_yolo_model, run_detection, process_detections, calculate_scale_factor
from src.camera.coordinate_calculation import calculate_navigation_command, create_turn_command, create_forward_command
from src.camera.camera_manager import CameraManager, draw_detection_box, draw_navigation_info, display_status
from src.communication.vision_commander import VisionCommander
from src.config.settings import *

def main():
    print("Loader YOLO model...")
    model = load_yolo_model()  # Starter kamera og YOLO model
    if model is None:
        return
    
    print("Initializing camera...")
    camera = CameraManager()
    if not camera.initialize_camera():
        return
    
    print("Initializing vision commander...")
    commander = VisionCommander()  # Sender kommandoer til EV3 robotten over netværk
    
    print("Starting main loop -  escape to exit")
    
    last_print_time = time.time()
    frame_count = 0
    
    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break
            
            frame_count += 1
            current_time = time.time()
            
            # Detekterer robot (head/tail) og bolde i real-time
            results = run_detection(model, frame)
            robot_head, robot_tail, balls, _ = process_detections(results, model)
            scale_factor = calculate_scale_factor(results, model)
            
            # Beregner navigation kommandoer baseret på robotens position
            navigation_info = None
            if robot_head and robot_tail and balls and scale_factor:
                closest_ball = min(balls, key=lambda b: 
                    ((robot_head["pos"][0] + robot_tail["pos"][0]) // 2 - b[0])**2 + 
                    ((robot_head["pos"][1] + robot_tail["pos"][1]) // 2 - b[1])**2
                )
                
                navigation_info = calculate_navigation_command(
                    robot_head, robot_tail, closest_ball, scale_factor
                )
                
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    if abs(angle_diff) > TURN_THRESHOLD:
                        turn_cmd = create_turn_command(angle_diff)
                        if turn_cmd:
                            commander.send_command(turn_cmd)
                    elif distance_cm > DISTANCE_THRESHOLD:
                        forward_cmd = create_forward_command(distance_cm)
                        if forward_cmd:
                            commander.send_command(forward_cmd)
            
            # Visualization
            display_frame = frame.copy()
            
            if robot_head:
                draw_detection_box(display_frame, robot_head["pos"], "robothead", CLASS_COLORS["robothead"])
            if robot_tail:
                draw_detection_box(display_frame, robot_tail["pos"], "robottail", CLASS_COLORS["robottail"])
            
            for ball_pos in balls:
                draw_detection_box(display_frame, ball_pos, "ball", CLASS_COLORS["white ball"])
            
            if navigation_info:
                draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    balls[0] if balls else None,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"]
                )
            
            display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            cv2.imshow("Robot Vision Navigation", display_frame)  # Viser live video med visualization
            
            # Status updates
            if current_time - last_print_time >= PRINT_INTERVAL:
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls),
                    scale_factor if scale_factor else 0
                ))
                last_print_time = current_time
            
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break
                
    except KeyboardInterrupt:
        print("camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 