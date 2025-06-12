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

# Global route manager
route_manager = RouteManager()

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
    last_gc_time = time.time()
    previous_ball_count = 0
    ball_delivery_threshold = 2
    delivering_to_goal = False
    
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
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
            # Tjek efter om der er samlet 2 bolde, så den skal aflevere
            if robot_head and robot_tail:
                current_ball_count = len(balls)
            else:
                current_ball_count = previous_ball_count

            if not delivering_to_goal:
                if previous_ball_count - current_ball_count >= ball_delivery_threshold:
                    goal_pos = get_cross_position(results, model)
                    if goal_pos:
                        print(" 2 balls collected — overriding route with goal position:", goal_pos)
                        route_manager.override_route_with_goal(goal_pos)
                        delivering_to_goal = True
                        previous_ball_count = current_ball_count  # Reset
            elif current_ball_count > previous_ball_count:
                # Reset after delivery if new balls appear (optional)
                delivering_to_goal = False

            previous_ball_count = current_ball_count
            # Simple vision-baseret navigation
            navigation_info = None
            
            # Tjek om mission er complete (ingen bolde)
            if robot_head and robot_tail and not balls:
                # Reset rute hvis ingen bolde er synlige (mission complete eller ny scene)
                if route_manager.route_created:
                    route_manager.reset_route()
                    
                if commander.can_send_command():
                    print("*** MISSION COMPLETE - NO BALLS VISIBLE - STOPPING ROBOT ***")
                    commander.send_stop_command()
            
            # RUTE-BASERET NAVIGATION: Følg fast rute i stedet for "nærmeste bold"
            elif robot_head and robot_tail and balls:
                # Beregn robot centrum for rute planlægning
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # Opret rute første gang vi ser bolde (med kollisionsundgåelse)
                route_manager.create_route_from_balls(balls, robot_center, walls, cross_pos)
                
                # Få nuværende mål fra ruten
                target_ball = route_manager.get_current_target()
                
                # Juster target for bolde tæt på vægge (vinkelret tilgang)
                if target_ball:
                    target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                else:
                    navigation_info = None
                
                if target_ball is None:
                    if delivering_to_goal:
                        print("Delivered to goal — resuming ball collection...")
                        delivering_to_goal = False
                        route_manager.reset_route()
                    else:
                        print(" *** ROUTE COMPLETE - ALL TARGETS VISITED ***")
                        if commander.can_send_command():
                            commander.send_stop_command()
                
                # Ny modulær navigation
                handle_robot_navigation(navigation_info, commander, route_manager)
            
            # Tegn rute og navigation visualization
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
            
            # Tilføj confidence threshold tekst
            cv2.putText(display_frame, "Conf: {:.2f}".format(CONFIDENCE_THRESHOLD), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            display_status(display_frame, robot_head, robot_tail, balls, scale_factor, navigation_info)
            
            # RUTE STATUS på skærm
            draw_route_status(display_frame, route_manager)
            cv2.imshow("YOLO OBB Detection", display_frame)
            
            # DISABLED: Explicit cleanup might be causing issues
            # del display_frame  
            # del frame
            
            # Status updates
            if current_time - last_print_time >= PRINT_INTERVAL:
                print("STATUS - Frame: {}, Robot: {}/{}, Balls: {}, Walls: {}, Scale: {:.2f}".format(
                    frame_count,
                    "YES" if robot_head else "NO",
                    "YES" if robot_tail else "NO", 
                    len(balls),
                    len(walls),
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
                
                # Print objekter koordinater
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