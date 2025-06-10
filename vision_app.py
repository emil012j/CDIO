# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
import math
import gc  # Tilføj garbage collection for memory management
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor
from src.camera.coordinate_calculation import calculate_navigation_command, create_turn_command, create_forward_command
from src.camera.camera_manager import CameraManager, draw_detection_box, draw_navigation_info, display_status
from src.communication.vision_commander import VisionCommander
from src.robot.progressive_navigator import ProgressiveNavigator
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
    
    print("Initializing progressive navigator...")
    navigator = ProgressiveNavigator()  # Intelligent boldtargeting med progressiv præcision
    
    print("Starting main loop -  escape to exit")
    
    last_print_time = time.time()
    frame_count = 0
    last_gc_time = time.time()  # Til garbage collection timing
    heavy_load_count = 0  # Tæl hvor mange frames med mange objekter
    last_heavy_load_time = time.time()  # Til at tracke hvor længe heavy load varer
    
    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break
            
            frame_count += 1
            current_time = time.time()
            
            # NUCLEAR OPTION: Hvis heavy load i mere end 30 sekunder, restart detection
            if heavy_load_count > 0 and current_time - last_heavy_load_time > 30.0:
                print("NUCLEAR OPTION: Heavy load too long - clearing everything and forcing cleanup")
                gc.collect()
                heavy_load_count = 0
                last_heavy_load_time = current_time
                # Force en pause
                time.sleep(0.1)
            
            # MEMORY MANAGEMENT: Skip frames hvis for mange objekter (forhindrer lag)
            total_objects_detected = 0
            
            # Detekterer robot (head/tail) og bolde i real-time
            results = run_detection(model, frame)
            scale_factor = calculate_scale_factor(results, model)
            
            # Tæl objekter FØRST for memory management
            if results and len(results) > 0 and hasattr(results[0], 'boxes') and results[0].boxes is not None:
                total_objects_detected = len(results[0].boxes)
            
            # AGGRESIV MEMORY MANAGEMENT: Hvis for mange objekter, KØR UDEN VISUALIZATION
            if total_objects_detected > MAX_OBJECTS_BEFORE_SKIP:
                heavy_load_count += 1
                if heavy_load_count == 1:  # Første gang heavy load
                    last_heavy_load_time = current_time
                print("HEAVY LOAD: {} objects detected, frame {} (no visualization)".format(total_objects_detected, heavy_load_count))
                
                # KØR KUN DETECTION - INGEN TEGNING/VISUALIZATION
                robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, None, scale_factor)
                
                # FORCE CLEANUP efter heavy load
                del results
                gc.collect()  # Force garbage collection efter hver heavy load frame
                
                # Simple navigation uden visualization
                if robot_head and robot_tail and balls and commander.can_send_command():
                    closest_ball = min(balls, key=lambda b: 
                        ((robot_head["pos"][0] + robot_tail["pos"][0]) // 2 - b[0])**2 + 
                        ((robot_head["pos"][1] + robot_tail["pos"][1]) // 2 - b[1])**2
                    )
                    
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, closest_ball, scale_factor)
                    
                    if navigation_info:
                        angle_diff = navigation_info["angle_diff"]
                        distance_cm = navigation_info["distance_cm"]
                        command, nav_info = navigator.get_navigation_command(angle_diff, distance_cm)
                        commander.send_command(command)
                        print("HEAVY LOAD NAVIGATION: {} - {:.1f}°, {:.1f}cm".format(nav_info["phase"], angle_diff, distance_cm))
                
                # Skip visualization helt
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
                
            else:
                # NORMAL PROCESSING med visualization
                heavy_load_count = 0  # Reset heavy load counter
            
            # Process detections og tegn på frame som i den gamle fil
            display_frame = frame.copy()
            robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
            # Tæl objekter for memory management
            total_objects_detected = (1 if robot_head else 0) + (1 if robot_tail else 0) + len(balls)
            
            # MEMORY MANAGEMENT: Spring frame over hvis for mange objekter (forhindrer overload)
            if total_objects_detected > MAX_OBJECTS_BEFORE_SKIP and frame_count % FRAME_SKIP_RATIO == 0:
                print("MEMORY: Skipping frame (too many objects: {})".format(total_objects_detected))
                # Explicit frame cleanup
                del display_frame, results
                continue
            
            # Simple vision-baseret navigation som den gamle fil
            navigation_info = None
            
            # Tjek om mission er complete (ingen bolde synlige)
            if robot_head and robot_tail and not balls:
                if commander.can_send_command():
                    print("*** MISSION COMPLETE - STOPPING ROBOT ***")
                    commander.send_stop_command()
            
            # Navigation kun hvis robot og bolde er synlige
            elif robot_head and robot_tail and balls:
                closest_ball = min(balls, key=lambda b: 
                    ((robot_head["pos"][0] + robot_tail["pos"][0]) // 2 - b[0])**2 + 
                    ((robot_head["pos"][1] + robot_tail["pos"][1]) // 2 - b[1])**2
                )
                
                navigation_info = calculate_navigation_command(
                    robot_head, robot_tail, closest_ball, scale_factor
                )
                
                # Intelligent navigation med progressiv navigator
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    # Få optimal kommando fra progressive navigator
                    command, nav_info = navigator.get_navigation_command(angle_diff, distance_cm)
                    phase = nav_info["phase"]
                    
                    # Vis detaljeret status
                    status_msg = navigator.get_status_message(phase, command, nav_info)
                    print("SMART NAVIGATION: {}".format(status_msg))
                    print("  -> Angle: {:.1f}°, Distance: {:.1f}cm, Phase: {}".format(
                        angle_diff, distance_cm, phase))
                    
                    # Send kommando til robot
                    commander.send_command(command)
            
            # Tegn robot retning og navigation linje som i den gamle fil
            if robot_head and robot_tail and balls:
                # Beregn robot centrum
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # Find nærmeste bold
                closest_ball = min(balls, key=lambda b: 
                    ((robot_head["pos"][0] + robot_tail["pos"][0]) // 2 - b[0])**2 + 
                    ((robot_head["pos"][1] + robot_tail["pos"][1]) // 2 - b[1])**2
                )
                
                # Tegn linje til målet som i den gamle fil
                cv2.line(display_frame, robot_center, closest_ball, (0, 255, 255), 2)
                
                # Tegn robot retning linje (tail→head extended) som i den gamle fil
                head_pos = robot_head["pos"] 
                tail_pos = robot_tail["pos"]
                dx = head_pos[0] - tail_pos[0]
                dy = head_pos[1] - tail_pos[1]
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    length = math.sqrt(dx*dx + dy*dy)
                    norm_dx = dx / length
                    norm_dy = dy / length
                    # Tegn linje fra tail gennem head og videre
                    end_x = int(head_pos[0] + norm_dx * 200)
                    end_y = int(head_pos[1] + norm_dy * 200)
                    cv2.line(display_frame, tail_pos, (end_x, end_y), (255, 0, 255), 2)
            
            if navigation_info:
                draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    balls[0] if balls else None,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"]
                )
            
            # Tilføj confidence threshold tekst som i den gamle fil
            cv2.putText(display_frame, "Conf: {:.2f}".format(CONFIDENCE_THRESHOLD), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            cv2.imshow("YOLO OBB Detection", display_frame)  # Samme titel som den gamle fil
            
            # MEMORY MANAGEMENT: Periodisk garbage collection 
            if current_time - last_gc_time >= MEMORY_GC_INTERVAL:
                gc.collect()  # Force garbage collection for at rydde op i memory
                last_gc_time = current_time
                print("MEMORY: Garbage collection performed (freed memory)")
            
            # AGGRESSIV MEMORY MANAGEMENT: Force cleanup hver 50. frame
            if frame_count % 50 == 0:
                gc.collect()
                print("AGGRESSIVE: Frame {} - forced garbage collection".format(frame_count))
            
            # MEMORY MANAGEMENT: Explicit frame cleanup efter hver iteration  
            del display_frame, results
            if 'closest_ball' in locals():
                del closest_ball
            
            # Status updates
            if current_time - last_print_time >= PRINT_INTERVAL:
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls),
                    scale_factor if scale_factor else 0
                ))
                
                # Debug information og robot status
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
                
                # Print objekter koordinater som i den gamle fil
                print("\n--- Objekter koordinater @ {} ---".format(time.strftime('%H:%M:%S')))
                if log_info_list:
                    for info in log_info_list:
                        print(info)
                else:
                    print(" Ingen objekter detekteret over tærsklen ({:.2f}).".format(CONFIDENCE_THRESHOLD))
                    
                last_print_time = current_time
            
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' som i den gamle fil
                break
                
    except KeyboardInterrupt:
        print("camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 