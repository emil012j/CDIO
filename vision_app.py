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

# BOLD LOCKING - vælg EN bold og hold fokus
target_ball = None  # Global variabel

def compute_detour_point(robot_center, ball_pos, cross_pos, avoid_r):
    """
    Beregn tangent-waypoint rundt om krydsets cirkel (radius=avoid_r).
    Returnerer det tangentpunkt, som minimerer total afstand R→T→B.
    """
    Rx, Ry = robot_center
    Cx, Cy = cross_pos
    Bx, By = ball_pos

    # afstand R→C og vinkel
    dx, dy = Cx-Rx, Cy-Ry
    d = math.hypot(dx, dy)
    if d <= avoid_r + 1e-6:
        # vi er allerede oveni – fallback til direkte bold
        return ball_pos

    theta = math.atan2(dy, dx)
    alpha = math.acos(avoid_r / d)

    candidates = []
    for sign in (+1, -1):
        ang = theta + sign*alpha
        Tx = Cx + avoid_r * math.cos(ang)
        Ty = Cy + avoid_r * math.sin(ang)
        total_dist = math.hypot(Tx-Rx, Ty-Ry) + math.hypot(Bx-Tx, By-Ty)
        candidates.append(((Tx, Ty), total_dist))

    # vælg punkt med kortest R→T→B
    detour_pt, _ = min(candidates, key=lambda x: x[1])
    return (int(detour_pt[0]), int(detour_pt[1]))

def choose_unblocked_target(robot_head, robot_tail, balls, cross_pos):
    """
    Returnerer enten:
      - den nærmeste bold, hvis path ikke krydser krydset, ell.
      - et omkørings-waypoint indtil videre, hvis direkte path er blokeret.
    """
    # robotcentrum i pixel
    rx = (robot_head["pos"][0] + robot_tail["pos"][0]) // 2
    ry = (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    robot_center = (rx, ry)

    # find nærmeste bold geometrisk
    balls_sorted = sorted(balls, key=lambda b: math.hypot(b[0]-rx, b[1]-ry))
    
    for b in balls_sorted:
        # tjek om kryds blokerer
        path = LineString([robot_center, b])
        if not cross_pos or path.distance(Point(cross_pos)) >= CROSS_AVOID_RADIUS:
            return b

    # alle boldene er blokeret – lav omkøringspunkt rundt om krydset ift. den nærmeste bold
    return compute_detour_point(robot_center, balls_sorted[0], cross_pos, CROSS_AVOID_RADIUS)

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
    
    # BOLD LOCKING: Fokuser på EN bold ad gangen
    target_ball = None
    
    last_print_time = time.time()
    frame_count = 0
    mission_started = False

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
            
            # Debug: vis antal bolde
            print(f"[DEBUG] Balls detected: {len(balls)} -> {balls}")
            if not mission_started and balls:
                mission_started = True

            # Stop kun hvis mission er startet og ingen bolde tilbage
            if robot_head and robot_tail and mission_started and not balls:
                if commander.can_send_command():
                    print("*** MISSION COMPLETE - STOPPING ROBOT ***")
                    commander.send_stop_command()
                break
            
            # Simple vision-baseret navigation
            navigation_info = None
            
            
            # Navigation kun hvis robot og target bold
            if robot_head and robot_tail and balls:
                # vælg en sikker target bold
                target_ball = choose_unblocked_target(robot_head, robot_tail, balls, cross_pos)
                navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                
                # Hvis vi er meget tæt på, bold er samlet - find ny target
                if navigation_info and navigation_info["distance_cm"] < DISTANCE_THRESHOLD:
                    print("Ball collected! Looking for next ball...")
                    
                
                # Simple navigation: TURN først til retning passer, så FORWARD
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    print("Navigation: Angle diff={:.1f}°, Distance={:.1f}cm".format(angle_diff, distance_cm))
                    
                    if abs(angle_diff) > TURN_THRESHOLD:
                        cmd = "right" if angle_diff > 0 else "left"
                        dur = abs(angle_diff) / ESTIMATED_TURN_RATE
                        print(f"DREJ {cmd} i {dur:.2f}s")
                        commander.send_turn_command(cmd, dur)
                    # SMÅ FORWARD STEP
                    elif distance_cm > SMALL_FORWARD_STEP_CM:
                        print(f"KØR {SMALL_FORWARD_STEP_CM}cm fremad")
                        commander.send_forward_command(SMALL_FORWARD_STEP_CM)
                    elif distance_cm > 0:
                        print(f"KØR {distance_cm:.1f}cm fremad")
                        commander.send_forward_command(distance_cm)
            
            # Tegn robot retning og navigation linje som i den gamle fil
            if robot_head and robot_tail and balls:
                # Beregn robot centrum
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # Find nærmeste bold
                closest_ball = choose_unblocked_target(robot_head, robot_tail, balls, cross_pos)
                
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