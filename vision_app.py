# -*- coding: utf-8 -*-
"""
hovedprogrammet der skal køre på pc'en
 Starter kamera+YOLO, detekterer robot+bolde, beregner navigation, sender kommandoer, viser live video
"""

import cv2
import time
import math
from typing import Optional, Dict, List, Tuple, Any
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw, calculate_scale_factor
from src.camera.coordinate_calculation import calculate_navigation_command
from src.camera.camera_manager import CameraManager
from src.communication.vision_commander import VisionCommander
from src.navigation.goal_navigation import GoalNavigation
from src.robot.controller import RobotController
from src.config.settings import *

class VisionCommanderAdapter(RobotController):
    """Adapter that makes VisionCommander compatible with RobotController interface"""
    def __init__(self, commander: VisionCommander):
        super().__init__()  # Initialize base RobotController
        self.commander = commander
        
    def send_turn_command(self, direction: str, duration: float) -> None:
        angle = duration * ESTIMATED_TURN_RATE
        if direction == "left":
            angle = -angle
        self.commander.send_turn_command(direction, duration)
        
    def send_forward_command(self, distance_cm: float) -> None:
        self.commander.send_forward_command(distance_cm)
        
    def stop_all_motors(self) -> None:
        self.commander.send_stop_command()
        
    def release_balls(self) -> None:
        # Send a special command to release balls
        self.commander.send_command({"command": "release_balls"})

def main():
    print("Loader YOLO model...")
    model = load_yolo_model()  # Starter kamera og YOLO model
    if model is None:
        return
    
    print("Initializing camera...")
    camera = CameraManager()
    if not camera.initialize_camera():
        return
    
    # Start track calibration
    print("\nStarting track calibration...")
    if not camera.calibrate_track():
        print("Track calibration failed or was cancelled")
        return
    print("Track calibration complete!")
    
    print("Initializing vision commander...")
    commander = VisionCommander()  # Sender kommandoer til EV3 robotten over netværk
    
    # Initialize goal navigation with adapter
    goal_nav = GoalNavigation(VisionCommanderAdapter(commander))
    goal_navigation_active = False
    
    print("Starting main loop - escape to exit (OPTIMERET VERSION)")
    
    last_print_time = time.time()
    frame_count = 0
    yolo_frame_count = 0
    
    # Cache til seneste detection resultater (bruger disse mellem YOLO kald)
    cached_results = None
    cached_robot_head: Optional[Dict[str, Any]] = None
    cached_robot_tail: Optional[Dict[str, Any]] = None
    cached_balls: List[Tuple[int, int]] = []
    cached_scale_factor: Optional[float] = None
    
    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break
            
            frame_count += 1
            current_time = time.time()
            
            # FRAME SKIPPING OPTIMIZATION: Kun kør YOLO på hver X frame!
            should_run_yolo = (frame_count % YOLO_FRAME_SKIP == 0)
            
            if should_run_yolo:
                # Detekterer robot (head/tail) og bolde - KUN på udvalgte frames
                yolo_frame_count += 1
                results = run_detection(model, frame)
                scale_factor = calculate_scale_factor(results, model)
                
                # Process detections og tegn på frame
                display_frame = frame.copy()
                robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
                
                # Cache resultaterne til brug i mellemliggende frames
                cached_results = results
                cached_robot_head = robot_head
                cached_robot_tail = robot_tail
                cached_balls = balls
                cached_scale_factor = scale_factor
            else:
                # Brug cached resultater og tegn kun UI overlay (meget hurtigere!)
                display_frame = frame.copy()
                robot_head = cached_robot_head
                robot_tail = cached_robot_tail
                balls = cached_balls
                scale_factor = cached_scale_factor
                
                # Tegn cached detection boxes (simpel version)
                if robot_head and "pos" in robot_head:
                    camera.draw_detection_box(display_frame, robot_head["pos"], "robot-head", (0, 0, 255))
                if robot_tail and "pos" in robot_tail:
                    camera.draw_detection_box(display_frame, robot_tail["pos"], "robot-tail", (255, 0, 255))
                for ball_pos in balls:
                    camera.draw_detection_box(display_frame, ball_pos, "ball", (0, 140, 255))
            
            # Navigation logic
            navigation_info = None
            
            if should_run_yolo:  # Navigation kun når vi har fresh data
                # Check if we should start goal navigation
                if not goal_navigation_active and robot_head and robot_tail and not balls:
                    print("*** NO BALLS VISIBLE - STARTING GOAL NAVIGATION ***")
                    goal_navigation_active = True
                    goal_nav.reset()  # Reset goal navigation state
                
                # Run appropriate navigation
                if goal_navigation_active:
                    # Goal navigation is active
                    if robot_head and robot_tail and "pos" in robot_head and "pos" in robot_tail:
                        if goal_nav.update(robot_head, robot_tail):
                            print("*** GOAL NAVIGATION COMPLETE ***")
                            goal_navigation_active = False
                            commander.send_stop_command()
                else:
                    # Normal ball collection navigation
                    if robot_head and robot_tail and balls and "pos" in robot_head and "pos" in robot_tail:
                        # Calculate robot center for finding closest ball
                        robot_center_x = (robot_head["pos"][0] + robot_tail["pos"][0]) // 2
                        robot_center_y = (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                        
                        closest_ball = min(balls, key=lambda b: 
                            (robot_center_x - b[0])**2 + 
                            (robot_center_y - b[1])**2
                        )
                        
                        navigation_info = calculate_navigation_command(
                            robot_head, robot_tail, closest_ball, scale_factor
                        )
                        
                        # Simple navigation: TURN først til retning passer, så FORWARD
                        if navigation_info and commander.can_send_command():
                            angle_diff = navigation_info["angle_diff"]
                            distance_cm = navigation_info["distance_cm"]
                            
                            print("Navigation: Angle diff={:.1f}°, Distance={:.1f}cm".format(angle_diff, distance_cm))
                            
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
            
            # Tegn robot retning og navigation linje (kun på fresh YOLO data)
            if should_run_yolo and robot_head and robot_tail and "pos" in robot_head and "pos" in robot_tail:
                # Beregn robot centrum
                robot_center_x = (robot_head["pos"][0] + robot_tail["pos"][0]) // 2
                robot_center_y = (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                robot_center = (robot_center_x, robot_center_y)
                
                if goal_navigation_active:
                    # Draw goal navigation state
                    cv2.putText(display_frame, f"GOAL NAV: {goal_nav.current_state}", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                elif balls:
                    # Find nærmeste bold
                    closest_ball = min(balls, key=lambda b: 
                        (robot_center_x - b[0])**2 + 
                        (robot_center_y - b[1])**2
                    )
                    
                    # Tegn linje til målet
                    cv2.line(display_frame, robot_center, closest_ball, (0, 255, 255), 2)
                
                # Tegn robot retning linje (tail→head extended)
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
                camera.draw_navigation_info(
                    display_frame,
                    navigation_info["robot_center"],
                    balls[0] if balls else None,
                    navigation_info["robot_heading"],
                    navigation_info["target_heading"]
                )
            
            # Tilføj confidence threshold tekst og performance info
            cv2.putText(display_frame, "Conf: {:.2f} | YOLO: {}/{}".format(
                CONFIDENCE_THRESHOLD, yolo_frame_count, frame_count), 
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Add goal navigation status
            if goal_navigation_active:
                cv2.putText(display_frame, "GOAL NAVIGATION ACTIVE", 
                          (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            camera.display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            cv2.imshow("YOLO OBB Detection - OPTIMERET", display_frame)
            
            # Status updates (mindre hyppigt for bedre performance)
            if current_time - last_print_time >= PRINT_INTERVAL:
                print("OPTIMERET STATUS - Frame: {}, YOLO: {}, Robot: {}/{}, Balls: {}, Scale: {:.2f}".format(
                    frame_count,
                    yolo_frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls) if balls else 0,
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
                    if goal_navigation_active:
                        print("*** GOAL NAVIGATION ACTIVE ***")
                    else:
                        print("*** INGEN BOLDE FUNDET - MISSION COMPLETE? ***")
                else:
                    print("*** {} BOLDE SYNLIGE ***".format(len(balls)))
                
                # Print performance info
                if frame_count > 0:
                    yolo_ratio = (yolo_frame_count / frame_count) * 100
                    print("*** PERFORMANCE: {:.1f}% YOLO frames (skipping arbejder!) ***".format(yolo_ratio))
                    
                last_print_time = current_time
            
            if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                break
                
    finally:
        camera.release()  # Use release() instead of cleanup()
        cv2.destroyAllWindows()
        print("Vision app stopped")

if __name__ == "__main__":
    main() 