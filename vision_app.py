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
from src.camera.camera_manager import CameraManager, draw_detection_box, draw_navigation_info, display_status
from src.communication.vision_commander import VisionCommander
from src.config.settings import *

def is_cross_blocking_path(robot_head, robot_tail, ball_pos, cross_pos):
    if not cross_pos:
        return False
    robot_x = (robot_head["pos"][0] + robot_tail["pos"][0]) // 2
    robot_y = (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    path = LineString([(robot_x, robot_y), ball_pos])
    return path.distance(Point(cross_pos)) < CROSS_AVOID_RADIUS

def choose_unblocked_ball(robot_head, robot_tail, balls, cross_pos):
    for ball in balls:
        if not is_cross_blocking_path(robot_head, robot_tail, ball, cross_pos):
            return ball
    return balls[0]  # fallback

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
    
    try:
        while True:
            frame = camera.read_frame()
            if frame is None:
                break
            
            frame_count += 1
            current_time = time.time()
            
            # DISABLED PERFORMANCE FEATURES - THEY MIGHT BE CAUSING LAG
            # if frame_count % 3 != 0:
            #     cv2.imshow("YOLO OBB Detection", frame)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break
            #     continue
            
            # if current_time - last_gc_time > 10:
            #     import gc
            #     gc.collect()
            #     last_gc_time = current_time
            
            # Detekterer robot (head/tail) og bolde i real-time
            results = run_detection(model, frame)
            scale_factor = calculate_scale_factor(results, model)
            cross_pos = get_cross_position(results, model)

            # Process detections og tegn på frame som i den gamle fil
            display_frame = frame.copy()
            robot_head, robot_tail, balls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
            # Simple vision-baseret navigation
            navigation_info = None
            
            # Tjek om mission er complete (ingen bolde)
            if robot_head and robot_tail and not balls:
                if commander.can_send_command():
                    print("*** MISSION COMPLETE - NO BALLS VISIBLE - STOPPING ROBOT ***")
                    commander.send_stop_command()
            
            # Navigation til nærmeste bold (stop 30 cm væk og udfør pickup sekvens)
            elif robot_head and robot_tail and balls:
                # Find nærmeste bold (ikke blokeret af kryds)
                target_ball = choose_unblocked_ball(robot_head, robot_tail, balls, cross_pos)
                navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                
                # Navigation: TURN først, så FORWARD til 30 cm, så pickup sekvens
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    print("Navigation: Angle diff={:.1f}deg, Distance={:.1f}cm".format(angle_diff, distance_cm))
                    
                    # PRECISE HITTING ZONE: Kun kør frem hvis angle_diff er i præcis rammezone
                    hitting_zone_min = -5.0  # Udvidet: Minimum præcis angle for at ramme bolden
                    hitting_zone_max = 10.0  # Udvidet: Maximum præcis angle for at ramme bolden
                    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
                    
                    # DEBUG: Show current state vs thresholds every frame
                    print("  DEBUG: Distance={:.1f}cm (≤29cm?), Angle={:.1f}° in [{:.1f}°,{:.1f}°]? = {}".format(
                        distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone))
                    
                    # TURN PHASE: Korriger vinkel hvis ikke i hitting zone (INGEN BEGRÆNSNING - op til 180°)
                    if not in_hitting_zone:  # Kun drej hvis IKKE i hitting zone
                        direction = "right" if angle_diff > hitting_zone_max else "left"
                        
                        # Beregn hvor meget der skal drejes for at komme i hitting zone
                        if angle_diff > hitting_zone_max:
                            turn_amount = angle_diff - hitting_zone_max  # Drej til max hitting zone
                        else:  # angle_diff < hitting_zone_min
                            turn_amount = hitting_zone_min - angle_diff  # Drej til min hitting zone
                        
                        # DIREKTE ROTATION: 90° = 0.5 rotations, 180° = 1.0 rotations
                        rotations = turn_amount / 90.0 * 0.5
                        print("ANGLE CORRECTION: {:.1f}deg -> hitting zone [{:.1f}, {:.1f}] -> {:.3f} rotations".format(
                            angle_diff, hitting_zone_min, hitting_zone_max, rotations))
                        commander.send_turn_rotation_command(direction, rotations)
                    
                    # FORWARD PHASE: Kun kør frem hvis i hitting zone og ikke for tæt på  
                    elif distance_cm > 29:  # Stop når vi er 29 cm væk for blind collection
                        move_distance = min(distance_cm - 29, 10)  # Kør til 29 cm væk, max 10 cm ad gangen
                        print("IN HITTING ZONE - DRIVING FORWARD {:.1f} cm (angle_diff={:.1f}deg PERFECT)".format(
                            move_distance, angle_diff))
                        commander.send_forward_command(move_distance)
                    
                    # BLIND BALL COLLECTION: I hitting zone OG ≤29 cm væk - start blind collection
                    else:
                        print("=== READY FOR BLIND COLLECTION ===")
                        print("Distance: {:.1f}cm ≤ 29cm, Angle: {:.1f}deg".format(distance_cm, angle_diff))
                        print("Hitting zone: [{:.1f}, {:.1f}], In zone: {}".format(
                            hitting_zone_min, hitting_zone_max, in_hitting_zone))
                        print("Robot position good: distance ≤ 29cm AND in hitting zone")
                        
                        if in_hitting_zone:
                            print("🎯 *** EXECUTING BLIND BALL COLLECTION *** 🎯")
                            success = commander.send_blind_ball_collection_command()
                            if success:
                                print("✅ Blind collection command sent successfully!")
                            else:
                                print("❌ Failed to send blind collection command!")
                        else:
                            print("❌ NOT IN HITTING ZONE - NEED ANGLE ADJUSTMENT FIRST")
                            print("   Angle {:.1f}° is outside [{:.1f}°, {:.1f}°]".format(
                                angle_diff, hitting_zone_min, hitting_zone_max))
            
            # Tegn robot retning og navigation linje
            if robot_head and robot_tail and balls:
                # Beregn robot centrum
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # Find nærmeste bold
                closest_ball = choose_unblocked_ball(robot_head, robot_tail, balls, cross_pos)
                
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
            cv2.imshow("YOLO OBB Detection", display_frame)
            
            # DISABLED: Explicit cleanup might be causing issues
            # del display_frame  
            # del frame
            
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