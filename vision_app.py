# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
import math
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor
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
            scale_factor = calculate_scale_factor(results, model)
            
            # Process detections og tegn på frame som i den gamle fil
            display_frame = frame.copy()
            robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
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
                
                # Simple navigation: TURN først til retning passer, så FORWARD
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    print("Navigation: Angle diff={:.1f}°, Distance={:.1f}cm".format(angle_diff, distance_cm))
                    
                    # TURN PHASE: Drej først til retningen er korrekt (større tolerance for første drejning)
                    if abs(angle_diff) > 15:  # Større threshold for at sikre ordentlig rotation
                        direction = "right" if angle_diff > 0 else "left"
                        # Større drejning til at komme i retning
                        turn_amount = min(abs(angle_diff), 45)  # Op til 45 grader ad gangen
                        duration = turn_amount / ESTIMATED_TURN_RATE
                        print("TURNING {} for {:.2f} seconds ({:.1f} degrees)".format(direction, duration, turn_amount))
                        commander.send_turn_command(direction, duration)
                    
                    # FORWARD PHASE: Kør frem når retningen er nogenlunde korrekt
                    elif distance_cm > 3:  # Kør helt tæt på for at samle boldene
                        move_distance = min(distance_cm, 20)  # Længere distance for at nå boldene
                        print("DRIVING FORWARD {:.1f} cm".format(move_distance))  
                        commander.send_forward_command(move_distance)
                    
                    # TARGET VERY CLOSE: Kort burst for at samle bolden
                    else:
                        print("TARGET VERY CLOSE - final push")
                        commander.send_forward_command(5)  # Kort fremad for at samle
            
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