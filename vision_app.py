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

# RUTE-BASERET NAVIGATION: Erstatter "nærmeste bold" med fast rute
class RouteManager:
    def __init__(self):
        self.route = []  # Liste af (x, y) koordinater
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0  # Tæl forsøg på nuværende target
        self.max_attempts = 3  # Max forsøg før vi giver op på en bold
        
    def create_route_from_balls(self, balls, robot_center, walls=None, cross_pos=None):
        """Lav en fast rute fra robot position til alle bolde med kollisionsundgåelse"""
        if self.route_created or not balls:
            return
            
        print("🗺️  CREATING BALL COLLECTION ROUTE...")
        
        # Filtrer bolde der er for tæt på vægge eller kors
        safe_balls = []
        if walls is None:
            walls = []
        
        for ball in balls:
            is_safe = True
            
            # Tjek afstand til vægge (undgå bolde tættere end 30 cm)
            for wall in walls:
                distance_to_wall = math.sqrt((ball[0] - wall[0])**2 + (ball[1] - wall[1])**2)
                if distance_to_wall < 150:  # 30 cm i pixels (ca 5 mm/px)
                    print("⚠️  Ball at ({}, {}) too close to wall at ({}, {}) - distance: {:.1f}px".format(
                        ball[0], ball[1], wall[0], wall[1], distance_to_wall))
                    is_safe = False
                    break
            
            # Tjek afstand til kors (undgå bolde tættere end 50 cm)
            if is_safe and cross_pos:
                distance_to_cross = math.sqrt((ball[0] - cross_pos[0])**2 + (ball[1] - cross_pos[1])**2)
                if distance_to_cross < 250:  # 50 cm i pixels
                    print("⚠️  Ball at ({}, {}) too close to cross at ({}, {}) - distance: {:.1f}px".format(
                        ball[0], ball[1], cross_pos[0], cross_pos[1], distance_to_cross))
                    is_safe = False
            
            if is_safe:
                safe_balls.append(ball)
        
        print("📍 Safe balls: {}/{}".format(len(safe_balls), len(balls)))
        
        if not safe_balls:
            print("❌ No safe balls found!")
            return
        
        # Start med robot position som udgangspunkt
        remaining_balls = list(safe_balls)  # Kopier listen
        route_points = []
        current_pos = robot_center
        
        # Simpel "nærmeste punkt" rute algoritme
        while remaining_balls:
            # Find nærmeste bold fra nuværende position
            distances = [math.sqrt((ball[0] - current_pos[0])**2 + (ball[1] - current_pos[1])**2) 
                        for ball in remaining_balls]
            nearest_index = distances.index(min(distances))
            nearest_ball = remaining_balls.pop(nearest_index)
            
            route_points.append(nearest_ball)
            current_pos = nearest_ball
            
        self.route = route_points
        self.current_target_index = 0
        self.route_created = True
        
        print("✅ ROUTE CREATED: {} waypoints".format(len(self.route)))
        for i, point in enumerate(self.route):
            print("   Point {}: ({}, {})".format(i+1, point[0], point[1]))
            
    def get_current_target(self):
        """Få nuværende mål i ruten"""
        if not self.route or self.current_target_index >= len(self.route):
            return None
        return self.route[self.current_target_index]
        
    def get_wall_approach_point(self, ball_pos, walls, scale_factor):
        """Beregn optimal tilgangspunkt for bold tæt på væg (vinkelret tilgang)"""
        if not walls or scale_factor is None:
            return ball_pos
            
        # Find nærmeste væg til bolden
        closest_wall = None
        min_distance = float('inf')
        
        for wall in walls:
            distance = math.sqrt((ball_pos[0] - wall[0])**2 + (ball_pos[1] - wall[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_wall = wall
        
        # Hvis bold er tættere end 75 px (ca 15 cm) til væg, beregn vinkelret tilgang
        if closest_wall and min_distance < 75:
            # Beregn vektor fra væg til bold
            wall_to_ball_x = ball_pos[0] - closest_wall[0]
            wall_to_ball_y = ball_pos[1] - closest_wall[1]
            
            # Normaliser vektor
            length = math.sqrt(wall_to_ball_x**2 + wall_to_ball_y**2)
            if length > 0:
                norm_x = wall_to_ball_x / length
                norm_y = wall_to_ball_y / length
                
                # Tilgangspunkt er 25 px (ca 5 cm) bag bolden i vinkelret retning fra væg
                approach_x = int(ball_pos[0] + norm_x * 25)  # 5 cm bag bolden
                approach_y = int(ball_pos[1] + norm_y * 25)
                
                print("🧱 WALL APPROACH: Ball at ({}, {}) near wall at ({}, {})".format(
                    ball_pos[0], ball_pos[1], closest_wall[0], closest_wall[1]))
                print("   → Approach point: ({}, {}) - 5cm behind ball, perpendicular to wall".format(approach_x, approach_y))
                
                return (approach_x, approach_y)
        
        return ball_pos
        
    def advance_to_next_target(self):
        """Gå til næste punkt i ruten"""
        self.current_target_index += 1
        self.collection_attempts = 0  # Reset attempts for new target
        print("🎯 ADVANCING TO NEXT TARGET: {}/{}".format(
            self.current_target_index + 1, len(self.route)))
            
    def increment_collection_attempts(self):
        """Øg antal forsøg på nuværende target"""
        self.collection_attempts += 1
        print("⚠️  Collection attempt {}/{} for current target".format(
            self.collection_attempts, self.max_attempts))
        return self.collection_attempts >= self.max_attempts
        
    def should_skip_current_target(self):
        """Tjek om vi skal give op på nuværende target"""
        return self.collection_attempts >= self.max_attempts
        
    def is_route_complete(self):
        """Tjek om ruten er færdig"""
        return self.current_target_index >= len(self.route)
        
    def reset_route(self):
        """Reset rute for ny mission"""
        self.route = []
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0
        print("🔄 ROUTE RESET")

# Global route manager
route_manager = RouteManager()

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
            robot_head, robot_tail, balls, walls, log_info_list = process_detections_and_draw(results, model, display_frame, scale_factor)
            
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
                
                if target_ball is None:
                    # Ingen flere mål - mission complete
                    print("🎉 *** ROUTE COMPLETE - ALL TARGETS VISITED ***")
                    if commander.can_send_command():
                        commander.send_stop_command()
                else:
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                
                # Navigation: TURN først, så FORWARD til 30 cm, så pickup sekvens
                if navigation_info and commander.can_send_command():
                    angle_diff = navigation_info["angle_diff"]
                    distance_cm = navigation_info["distance_cm"]
                    
                    print("Navigation: Angle diff={:.1f}deg, Distance={:.1f}cm".format(angle_diff, distance_cm))
                    
                    # PRECISE HITTING ZONE: Kun kør frem hvis angle_diff er i præcis rammezone
                    hitting_zone_min = -3.0  # Strammere: Minimum præcis angle for at ramme bolden
                    hitting_zone_max = 3.0   # Strammere: Maximum præcis angle for at ramme bolden
                    in_hitting_zone = hitting_zone_min <= angle_diff <= hitting_zone_max
                    
                    # DEBUG: Show current state vs thresholds every frame
                    print("  DEBUG: Distance={:.1f}cm (≤29cm?), Angle={:.1f}° in [{:.1f}°,{:.1f}°]? = {}".format(
                        distance_cm, angle_diff, hitting_zone_min, hitting_zone_max, in_hitting_zone))
                    
                    # TURN PHASE: Korriger vinkel hvis ikke i hitting zone (INGEN BEGRÆNSNING - op til 180°)
                    if not in_hitting_zone:  # Kun drej hvis IKKE i hitting zone
                        # Tjek om vi har prøvet for mange gange på denne target
                        if route_manager.should_skip_current_target():
                            print("🚫 TOO MANY ATTEMPTS ON THIS TARGET - SKIPPING")
                            route_manager.advance_to_next_target()
                        else:
                            direction = "right" if angle_diff > hitting_zone_max else "left"
                            
                            # Beregn hvor meget der skal drejes for at komme i hitting zone
                            if angle_diff > hitting_zone_max:
                                turn_amount = angle_diff - hitting_zone_max  # Drej til max hitting zone
                            else:  # angle_diff < hitting_zone_min
                                turn_amount = hitting_zone_min - angle_diff  # Drej til min hitting zone
                            
                            # KORRIGERET ROTATION: 180° = 0.5 rotations (mere realistisk for EV3)
                            rotations = turn_amount / 180.0 * 0.5
                            
                            # BEGRÆNS ROTATION: Max 0.35 rotations (63°) ad gangen - øget for hurtigere drejning
                            max_rotations = 0.35
                            if rotations > max_rotations:
                                rotations = max_rotations
                                print("WARNING: Rotation begranset til {:.3f} (var {:.3f})".format(max_rotations, turn_amount / 180.0 * 0.5))
                            
                            print("ANGLE CORRECTION: {:.1f}deg -> hitting zone [{:.1f}, {:.1f}] -> {:.3f} rotations".format(
                                angle_diff, hitting_zone_min, hitting_zone_max, rotations))
                            commander.send_turn_rotation_command(direction, rotations)
                    
                    # FORWARD PHASE: Kun kør frem hvis i hitting zone og ikke for tæt på  
                    elif distance_cm > 29:  # Stop når vi er 29 cm væk for blind collection
                        move_distance = min(distance_cm - 29, 8)  # HURTIGERE: Øget fra 5 til 8 cm ad gangen for hurtigere fremgang
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
                                # GÅ TIL NÆSTE PUNKT I RUTEN efter collection
                                route_manager.advance_to_next_target()
                            else:
                                print("❌ Failed to send blind collection command!")
                                # Øg antal forsøg og tjek om vi skal give op
                                should_skip = route_manager.increment_collection_attempts()
                                if should_skip:
                                    print("🚫 MAX ATTEMPTS REACHED - SKIPPING TO NEXT TARGET")
                                    route_manager.advance_to_next_target()
                        else:
                            print("❌ NOT IN HITTING ZONE - NEED ANGLE ADJUSTMENT FIRST")
                            print("   Angle {:.1f}° is outside [{:.1f}°, {:.1f}°]".format(
                                angle_diff, hitting_zone_min, hitting_zone_max))
            
            # Tegn robot retning og navigation linje
            if robot_head and robot_tail:
                # Beregn robot centrum
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                
                # TEGN RUTE: Vis hele ruten og nuværende mål
                current_target = route_manager.get_current_target()
                if current_target:
                    # Tegn linje til nuværende mål (GUL)
                    cv2.line(display_frame, robot_center, current_target, (0, 255, 255), 3)
                    # Tegn nuværende mål som stor cirkel (GUL)
                    cv2.circle(display_frame, current_target, 15, (0, 255, 255), 3)
                    cv2.putText(display_frame, "TARGET {}".format(route_manager.current_target_index + 1), 
                               (current_target[0] + 20, current_target[1] - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Tegn hele ruten som farvede cirkler
                for i, waypoint in enumerate(route_manager.route):
                    if i < route_manager.current_target_index:
                        # Besøgte punkter: GRØN
                        cv2.circle(display_frame, waypoint, 8, (0, 255, 0), -1)
                        cv2.putText(display_frame, "✓", (waypoint[0] - 5, waypoint[1] + 5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    elif i == route_manager.current_target_index:
                        # Nuværende mål: allerede tegnet ovenfor
                        pass
                    else:
                        # Fremtidige mål: BLÅ
                        cv2.circle(display_frame, waypoint, 8, (255, 0, 0), 2)
                        cv2.putText(display_frame, str(i + 1), (waypoint[0] - 5, waypoint[1] + 5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                
                # TEGN VÆGGE (kun markering, ingen cirkler da vægge er aflange)
                for wall in walls:
                    # Væg som lille rød firkant
                    cv2.rectangle(display_frame, (wall[0]-10, wall[1]-10), (wall[0]+10, wall[1]+10), (0, 0, 255), -1)
                    cv2.putText(display_frame, "WALL", (wall[0] + 15, wall[1]), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
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
            
            # RUTE STATUS på skærm
            if route_manager.route:
                route_text = "ROUTE: {}/{} waypoints (attempts: {}/{})".format(
                    route_manager.current_target_index + 1, len(route_manager.route),
                    route_manager.collection_attempts, route_manager.max_attempts)
                cv2.putText(display_frame, route_text, (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
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