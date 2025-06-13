# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
import math
from shapely.geometry import LineString, Point
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor, get_cross_position
from src.camera.coordinate_calculation import calculate_navigation_command, create_turn_command, create_forward_command
from src.camera.camera_manager import CameraManager
from src.communication.vision_commander import VisionCommander
from src.camera.Goal_Calibrator import GoalCalibrator
from src.camera.Goal_utils import GoalUtils
from src.config.settings import *


def main():
    # Goal calibration setup
    print("Setting up goal position...")
    calibrator = GoalCalibrator()
    goal_utils = GoalUtils()
    
    # Check if goal position exists, if not start calibration
    goal_position = goal_utils.get_goal_position()
    if goal_position:
        print("Goal position loaded: {}".format(goal_position))
    else:
        print("No goal position found - calibration required!")
        print("You will need to calibrate the goal position before starting the mission.")
        print("Press 'c' during operation to start goal calibration.")
    
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
    
    # Multi-cycle state management
    COLLECTING = "collecting"
    DELIVERING = "delivering"
    RELEASING = "releasing"
    COMPLETE = "complete"
    
    current_state = COLLECTING
    TOTAL_BALLS_ON_COURT = 11   # Total balls to collect
    STORAGE_CAPACITY = 6        # Max balls robot can carry per trip
    current_run_balls = 0       # Balls collected in current run
    total_balls_collected = 0   # Total balls collected across all runs
    previous_ball_count = 0     # For detecting when balls are collected
    
    print("*** MULTI-CYCLE BALL COLLECTION MISSION ***")
    print("Target: {} balls total, {} balls per trip".format(TOTAL_BALLS_ON_COURT, STORAGE_CAPACITY))
    
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
            cross_pos = get_cross_position(results, model)

            # Process detections og tegn på frame som i den gamle fil
            display_frame = frame.copy()
            robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
            # Multi-cycle navigation logic
            navigation_info = None
            
            # Ball counting logic - detect when balls are collected
            current_visible_balls = len(balls)
            if current_visible_balls < previous_ball_count:
                balls_difference = previous_ball_count - current_visible_balls
                current_run_balls += balls_difference
                total_balls_collected += balls_difference
                print("*** BALL(S) COLLECTED! Run: {}/{}, Total: {}/{} ***".format(
                    current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
            previous_ball_count = current_visible_balls
            
            # State machine navigation
            closest_ball = None  # Initialize to avoid undefined variable errors
            
            if robot_head and robot_tail:
                
                if current_state == COLLECTING:
                    # Check if storage is full or no more balls visible
                    if current_run_balls >= STORAGE_CAPACITY:
                        current_state = DELIVERING
                        print("*** STORAGE FULL - SWITCHING TO GOAL DELIVERY ***")
                    elif not balls and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                        current_state = COMPLETE
                        print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    elif balls:
                         # Continue with existing ball collection logic
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
                             
                             print("COLLECTING: Angle diff={:.1f}°, Distance={:.1f}cm".format(angle_diff, distance_cm))
                             
                             # TURN PHASE: Drej først til retningen er korrekt - PRÆCIST første gang
                             if abs(angle_diff) > 10:  # Reduceret threshold for mere præcis navigation
                                 direction = "right" if angle_diff > 0 else "left"
                                 # Drej hele vinklen på én gang for præcision
                                 turn_amount = abs(angle_diff)  # Fjernet 45° begrænsning - drej præcist!
                                 duration = turn_amount / ESTIMATED_TURN_RATE
                                 print("PRECISE TURNING {} for {:.2f} seconds ({:.1f} degrees)".format(direction, duration, turn_amount))
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
                
                elif current_state == DELIVERING:
                    # Goal navigation using existing coordinate calculation system
                   
                    goal_position = goal_utils.get_goal_position()
                    if goal_position and commander.can_send_command():
                        print("*** INITIATING GOAL DELIVERY COMMAND ***")

                        tail_px = robot_tail["pos"]
                        head_px = robot_head["pos"]
                        goal_px = goal_position  # Already in pixel coordinates

                        # Send full delivery command via VisionCommander
                        commander.send_goal_delivery_command(tail=tail_px, head=head_px, goal=goal_px)

                        current_state = RELEASING  # Can also delay this if needed
                    else:
                        print("ERROR: No goal position set - cannot navigate to goal!")
                    
                elif current_state == RELEASING:
                    # Simple ball release using existing robot mechanism
                    if commander.can_send_command():
                        print("*** RELEASING BALLS - RUNNING MOTOR C AT SPEED 30 ***")
                        commander.send_release_command()
                        current_run_balls = 0  # Reset storage for next run
                        
                        if total_balls_collected >= TOTAL_BALLS_ON_COURT:
                            current_state = COMPLETE
                            print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                        else:
                            current_state = COLLECTING
                            print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN ***")
                    
                elif current_state == COMPLETE:
                    if commander.can_send_command():
                        print("*** MISSION COMPLETE - STOPPING ROBOT ***")
                        commander.send_stop_command()
            
            # Tegn robot retning og navigation linje
            if robot_head and robot_tail:
                # Beregn robot centrum
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # Draw navigation line based on current state
                if current_state == COLLECTING and balls and closest_ball:
                    # Yellow line to closest ball
                    cv2.line(display_frame, robot_center, closest_ball, (0, 255, 255), 2)
                elif current_state == DELIVERING:
                    # Blue line to goal
                    goal_pos = goal_utils.get_goal_position()
                    if goal_pos:
                        cv2.line(display_frame, robot_center, goal_pos, (255, 0, 0), 3)  # Thicker blue line
                
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
                # Determine target for navigation info display
                target_pos = None
                if current_state == COLLECTING and balls:
                    target_pos = balls[0]
                elif current_state == DELIVERING:
                    target_pos = goal_utils.get_goal_position()
                
                camera.draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    target_pos,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"]
                )
            
            # Tilføj confidence threshold tekst som i den gamle fil
            cv2.putText(display_frame, "Conf: {:.2f}".format(CONFIDENCE_THRESHOLD), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Add state information display
            cv2.putText(display_frame, "State: {}".format(current_state.upper()), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, "Run: {}/{}".format(current_run_balls, STORAGE_CAPACITY), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_frame, "Total: {}/{}".format(total_balls_collected, TOTAL_BALLS_ON_COURT), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw goal position if available
            current_goal_pos = goal_utils.get_goal_position()
            if current_goal_pos:
                cv2.circle(display_frame, current_goal_pos, 15, (255, 0, 0), -1)  # Blue circle for goal
                cv2.putText(display_frame, "GOAL", (current_goal_pos[0]+20, current_goal_pos[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            else:
                cv2.putText(display_frame, "No Goal Set - Press 'c' to calibrate", (10, 140), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            camera.display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
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
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # 'q' to quit
                break
            elif key == ord('c'):  # 'c' to calibrate goal
                print("*** STARTING GOAL CALIBRATION ***")
                calibrator.start_calibration(frame)
                # Reload goal position after calibration
                goal_utils.load_goal()  # Refresh from file
                goal_position = goal_utils.get_goal_position()
                if goal_position:
                    print("Goal calibration complete: {}".format(goal_position))
                else:
                    print("Goal calibration was cancelled or failed")
                
    except KeyboardInterrupt:
        print("camera released")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 