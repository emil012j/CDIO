"""
References:
-----------
1. YOLO Model Training in Colab:
   - Author: Evan Juras (2025)
   - Source: "Train_YOLO_Models.ipynb" (GitHub)
   - URL: https://github.com/EdjeElectronics/Train-and-Deploy-YOLO-Models
   - Used for: 
     * Training custom YOLO models
     * Dataset preparation (Label Studio integration)
     * Model deployment examples

2. Ultralytics YOLO Documentation:
   - Source: Ultralytics Official Docs
   - URL: https://docs.ultralytics.com/
   - Used for:
     * YOLO model configuration (data.yaml)
     * Inference and training commands

3. OpenCV Python Reference:
   - Source: OpenCV 4.x Documentation
   - URL: https://docs.opencv.org/4.x/
   - Used for:
     * Video capture (cv2.VideoCapture)
     * Image processing (blur, edge detection)
     * Coordinate system visualization
     
4. Data Labeling Tool:
   - Tool: Label Studio
   - URL: https://labelstud.io/
   - Used for: Image annotation for training dataset
"""
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math
import os
import socket
import json
import threading

# Pythagoras til beregning af afstand mellem to punkter
def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

MODEL_PATH = "best.pt"
SOURCE = 1
RESOLUTION = (1280, 720)
CONF_THRESH_DISPLAY = 0.35

BOLD_DIAMETER_MM = 40
CROSS_DIAMETER_MM = 200.0
EGG_SIZE_THRESHOLD_MM = 58.0

CLASS_COLORS = { "cross": (0, 255, 0), "egg": (255, 0, 0), "orange ball": (0, 140, 255), "white ball": (255, 255, 255), "robothead": (0, 0, 255), "robottail": (255, 0, 255) }
ORIENTED_OBJECTS = ["robothead", "robottail", "egg", "cross"]
EGG_OBB_ASPECT_RATIO_THRESHOLD = 1.3


ROBOT_IP = "169.254.71.123"  
COMMAND_PORT = 1233

def get_obb_dimensions_from_obb_results(obb_results, index):
    try:
        if hasattr(obb_results, 'wh') and obb_results.wh is not None and len(obb_results.wh) > index:
             wh = obb_results.wh[index].cpu().numpy(); w, h = min(wh), max(wh); return float(w), float(h)
        elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
            points = obb_results.xyxyxyxy[index].cpu().numpy().reshape(4, 2)
            side1=np.linalg.norm(points[0]-points[1]); side2=np.linalg.norm(points[1]-points[2])
            w, h = min(side1, side2), max(side1, side2); return float(w), float(h)
        else: return None, None
    except Exception: return None, None

def is_likely_egg_by_obb_shape_idx(obb_results, index):
    try:
        obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(obb_results, index)
        if obb_w_px is not None and obb_h_px is not None and obb_w_px > 1e-3:
            aspect_ratio = obb_h_px / obb_w_px
            return aspect_ratio >= EGG_OBB_ASPECT_RATIO_THRESHOLD
        else: return False
    except Exception: return False

def get_class_id(class_name_lower, model):
    for idx, name in model.names.items():
        if name.lower() == class_name_lower: return idx
    return None

def calculate_scale_factor(results, model):
    try:
        if results is None or not hasattr(results, 'obb') or results.obb is None: return None
        if not hasattr(results.obb, 'cls') or results.obb.cls is None: return None
        num_obb_boxes = len(results.obb.cls)
        if num_obb_boxes == 0: return None
        cross_id_target = get_class_id('cross', model)
        if cross_id_target is None: return None
        for i in range(num_obb_boxes):
             cls_id = int(results.obb.cls[i])
             conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
             if cls_id == cross_id_target and conf >= CONF_THRESH_DISPLAY:
                obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(results.obb, i)
                if obb_w_px is not None and obb_h_px is not None:
                    cross_obb_size_px = (obb_w_px + obb_h_px) / 2.0
                    if cross_obb_size_px > 1: return CROSS_DIAMETER_MM / cross_obb_size_px
        return None
    except Exception: return None

def calculate_orientation(obb_results, index, label):
    try:
        if hasattr(obb_results, 'angle') and obb_results.angle is not None and len(obb_results.angle)>index and obb_results.angle[index] is not None: return float(obb_results.angle[index])
        elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
           points=obb_results.xyxyxyxy[index].cpu().numpy().reshape(4,2); s1=np.linalg.norm(points[0]-points[1]); s2=np.linalg.norm(points[1]-points[2])
           if s1>s2: dx=points[1][0]-points[0][0]; dy=points[1][1]-points[0][1]
           else: dx=points[2][0]-points[1][0]; dy=points[2][1]-points[1][1]
           return math.degrees(math.atan2(-dy, dx))
        elif hasattr(obb_results, 'xyxy') and obb_results.xyxy is not None and len(obb_results.xyxy) > index:
            x1,y1,x2,y2=map(int,obb_results.xyxy[index]); w=x2-x1; h=y2-y1
            if h>0 and w>0 and label in ["egg","cross"] and (w/h>1.2 or h/w>1.2): return 90.0 if h>w else 0.0
        return 0.0
    except Exception: return 0.0

def draw_robot_tangent_line(frame, tail_pos, head_pos, color=(0,255,255), thickness=1, line_length=2000):
    try:
        if tail_pos is None or head_pos is None: return
        dx=head_pos[0]-tail_pos[0]; dy=head_pos[1]-tail_pos[1]
        if abs(dx)<1e-6 and abs(dy)<1e-6: return
        length=math.sqrt(dx*dx+dy*dy);
        if length<1e-6: return
        norm_dx=dx/length; norm_dy=dy/length
        sx=int(tail_pos[0]); sy=int(tail_pos[1])
        ex=int(head_pos[0]+norm_dx*line_length); ey=int(head_pos[1]+norm_dy*line_length)
        cv2.line(frame, (sx,sy), (ex,ey), color, thickness)
    except Exception: pass

def calculate_robot_heading(tail_pos, head_pos):
    """Calculate robot's current heading angle from tail to head"""
    dx = head_pos[0] - tail_pos[0]
    dy = head_pos[1] - tail_pos[1]
    # Calculate angle in degrees (0° = right, positive = counterclockwise)
    angle = math.degrees(math.atan2(-dy, dx))  # Negative dy for image coordinates
    return angle

def calculate_target_heading(robot_center, target_pos):
    """Calculate desired heading angle from robot to target"""
    dx = target_pos[0] - robot_center[0]
    dy = target_pos[1] - robot_center[1]
    # Calculate angle in degrees
    angle = math.degrees(math.atan2(-dy, dx))  # Negative dy for image coordinates
    return angle

def normalize_angle(angle):
    """Normalize angle to [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def send_vision_guided_command(robot_head, robot_tail, target_pos, scale_factor):
    """Send simple turn and move commands using computer vision feedback"""
    def send_command():
        print("\nUsing computer vision guidance to navigate to ball...")
        try:
            # Calculate robot center and current heading
            robot_center = (
                (robot_head[0] + robot_tail[0]) // 2,
                (robot_head[1] + robot_tail[1]) // 2
            )
            
            current_heading = calculate_robot_heading(robot_tail, robot_head)
            target_heading = calculate_target_heading(robot_center, target_pos)
            
            # Calculate angle difference (how much to turn)
            angle_diff = normalize_angle(target_heading - current_heading)
            
            # Calculate distance to target
            dx_pixels = target_pos[0] - robot_center[0]
            dy_pixels = target_pos[1] - robot_center[1]
            distance_pixels = math.sqrt(dx_pixels*dx_pixels + dy_pixels*dy_pixels)
            distance_cm = (distance_pixels * scale_factor) / 10.0  # Convert to cm
            
            print("Robot heading: {:.1f}°, Target heading: {:.1f}°".format(current_heading, target_heading))
            print("Need to turn: {:.1f}°, Distance: {:.1f} cm".format(angle_diff, distance_cm))
            
            # Small incremental movements - no big turns!
            if abs(angle_diff) > 10:  # Only turn if more than 10 degrees off
                # Small turn increments - max 30 degrees at a time
                turn_amount = 30 if angle_diff > 0 else -30
                if abs(angle_diff) < 30:
                    turn_amount = angle_diff
                    
                print("Sending small turn command: {:.1f} degrees (target: {:.1f})".format(turn_amount, angle_diff))
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ROBOT_IP, COMMAND_PORT))
                
                turn_command = {
                    "command": "simple_turn",
                    "direction": "right" if turn_amount > 0 else "left",
                    "duration": abs(turn_amount) / 180.0  # Faster turning - half the time
                }
                
                sock.send(json.dumps(turn_command).encode())
                sock.close()
                print("Small turn command sent")
                
            elif distance_cm > 5:  # Move if more than 5cm away  
                # Limit forward movement to reasonable distance
                move_distance = min(distance_cm, 20.0)  # Smaller moves for faster response
                
                print("Sending forward command: {:.1f} cm".format(move_distance))
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((ROBOT_IP, COMMAND_PORT))
                
                move_command = {
                    "command": "forward", 
                    "distance": move_distance
                }
                
                sock.send(json.dumps(move_command).encode())
                sock.close()
                print("Forward command sent")
            else:
                print("Robot is already aimed correctly and close to target")
                
        except Exception as e:
            print("Could not send vision-guided command: {}".format(e))
    
    # Start command in separate thread
    threading.Thread(target=send_command, daemon=True).start()

def main():
    print("Ball Detection & Robot Navigation System")
    print("========================================")
    print("Robot IP: {} (change ROBOT_IP variable if different)".format(ROBOT_IP))
    
    try:
        if not os.path.exists(MODEL_PATH): 
            print("ERROR: Cannot find {}".format(MODEL_PATH))
            return
        print("Loading YOLO model...")
        model = YOLO(MODEL_PATH)
        if get_class_id('cross', model) is None: 
            print("WARNING: Model missing 'cross' class - scale detection may not work")
    except Exception as e: 
        print("ERROR loading model: {}".format(e))
        return

    print("Opening camera...")
    cap = cv2.VideoCapture(SOURCE, cv2.CAP_DSHOW)
    if not cap.isOpened(): 
        print("Trying backup camera...")
        cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened(): 
        print("ERROR: Could not open camera!")
        return
    
    print("Setting camera resolution...")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)

    print("System ready! Press 'q' to quit")
    print("Looking for robot (robothead + robottail) and balls...")
    
    last_print_time = time.time()
    print_interval = 3
    last_command_time = 0
    command_cooldown = 0.5  # Faster commands - 0.5 seconds between commands

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: 
                time.sleep(0.1)
                continue

            # Run YOLO detection
            results = None
            try:
                results_list = model(frame, imgsz=640, conf=CONF_THRESH_DISPLAY, verbose=False)
                if isinstance(results_list, list) and len(results_list) > 0:
                    results = results_list[0]
            except Exception: 
                pass

            display_frame = frame.copy()
            
            # Draw crosshairs
            cv2.line(display_frame, (RESOLUTION[0]//2,0), (RESOLUTION[0]//2,RESOLUTION[1]), (0,255,0), 1)
            cv2.line(display_frame, (0,RESOLUTION[1]//2), (RESOLUTION[0],RESOLUTION[1]//2), (0,255,0), 1)

            # Process detections
            process_detections = False
            num_detections = 0
            if results is not None and hasattr(results, 'obb') and results.obb is not None:
                if hasattr(results.obb, 'cls') and results.obb.cls is not None:
                    try: 
                        num_detections = len(results.obb.cls)
                        process_detections = num_detections > 0
                    except TypeError: 
                        pass

            scale = None
            if process_detections: 
                scale = calculate_scale_factor(results, model)

            robot_head = None
            robot_tail = None
            balls = []

            if process_detections:
                for i in range(num_detections):
                    try:
                        cls_id = int(results.obb.cls[i])
                        conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
                        if cls_id < 0 or cls_id >= len(model.names): 
                            continue
                        
                        label = model.names[cls_id]
                        
                        # Get position
                        cx_calc, cy_calc = 0, 0
                        x1, y1, x2, y2 = 0, 0, 0, 0
                        
                        if hasattr(results.obb, 'xyxyxyxy') and results.obb.xyxyxyxy is not None:
                            if len(results.obb.xyxyxyxy) > i:
                                obb_points = results.obb.xyxyxyxy[i].cpu().numpy().reshape(4, 2).astype(np.int32)
                                x_coords = obb_points[:,0]
                                y_coords = obb_points[:,1]
                                x1, y1 = np.min(x_coords), np.min(y_coords)
                                x2, y2 = np.max(x_coords), np.max(y_coords)
                                cx_calc = int(np.mean(x_coords))
                                cy_calc = int(np.mean(y_coords))
                                
                                # Draw detection
                                color = CLASS_COLORS.get(label, (128, 128, 128))
                                cv2.polylines(display_frame, [obb_points], isClosed=True, color=color, thickness=2)
                        elif hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None:
                            if len(results.obb.xyxy) > i:
                                x1, y1, x2, y2 = map(int, results.obb.xyxy[i])
                                cx_calc, cy_calc = (x1+x2)//2, (y1+y2)//2
                                
                                # Draw detection
                                color = CLASS_COLORS.get(label, (128, 128, 128))
                                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                        else:
                            continue
                            
                        # Label the detection
                        color = CLASS_COLORS.get(label, (128, 128, 128))
                        label_text = "{} ({:.2f})".format(label, conf)
                        cv2.putText(display_frame, label_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
                        cv2.putText(display_frame, label_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                        
                        # Store robot parts and balls
                        if label == "robothead":
                            robot_head = {"pos": (cx_calc, cy_calc)}
                        elif label == "robottail":
                            robot_tail = {"pos": (cx_calc, cy_calc)}
                        elif "ball" in label.lower() and "egg" not in label.lower():
                            balls.append((cx_calc, cy_calc))
                            
                    except Exception as e:
                        print("Error processing detection {}: {}".format(i, e))

            # Vision-guided robot navigation logic
            current_time = time.time()
            if robot_head and robot_tail and balls and scale:
                if current_time - last_command_time >= command_cooldown:
                    try:
                        # Calculate robot center
                        robot_pos = (
                            (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                            (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                        )
                        
                        # Find nearest ball
                        closest_ball = min(balls, key=lambda b: calculate_distance(robot_pos, b))
                        
                        # Calculate and display robot heading
                        current_heading = calculate_robot_heading(robot_tail["pos"], robot_head["pos"])
                        target_heading = calculate_target_heading(robot_pos, closest_ball)
                        angle_diff = normalize_angle(target_heading - current_heading)
                        
                        # Draw navigation info
                        cv2.line(display_frame, robot_tail["pos"], robot_head["pos"], (255, 0, 255), 3)  # Robot direction
                        cv2.line(display_frame, robot_pos, closest_ball, (0, 255, 255), 3)  # Target line
                        cv2.circle(display_frame, robot_pos, 10, (255, 0, 255), -1)  # Robot center
                        cv2.circle(display_frame, closest_ball, 15, (0, 255, 255), 3)  # Target ball
                        
                        # Display heading info
                        cv2.putText(display_frame, "Robot: {:.0f}° Target: {:.0f}° Turn: {:.0f}°".format(
                            current_heading, target_heading, angle_diff), 
                            (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        
                        # Send vision-guided command to robot
                        send_vision_guided_command(robot_head["pos"], robot_tail["pos"], closest_ball, scale)
                        last_command_time = current_time
                        
                        cv2.putText(display_frame, "VISION-GUIDED NAVIGATION", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                    except Exception as e:
                        print("Error in vision-guided navigation: {}".format(e))
            else:
                # Show what's missing
                status_y = 80
                if not robot_head or not robot_tail:
                    cv2.putText(display_frame, "NEED ROBOT (head+tail)", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    status_y += 25
                if not balls:
                    cv2.putText(display_frame, "NEED BALLS", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    status_y += 25
                if not scale:
                    cv2.putText(display_frame, "NEED SCALE (cross)", (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Status information
            cv2.putText(display_frame, "Conf: {:.2f}".format(CONF_THRESH_DISPLAY), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if scale:
                cv2.putText(display_frame, "Scale: {:.2f} mm/px".format(scale), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "No scale reference", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.putText(display_frame, "Detections: {}".format(num_detections), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Periodic status print
            if time.time() - last_print_time > print_interval:
                print("\n--- Status @ {} ---".format(time.strftime('%H:%M:%S')))
                print("Robot head: {}, Robot tail: {}".format("YES" if robot_head else "NO", "YES" if robot_tail else "NO"))
                print("Balls found: {}".format(len(balls)))
                print("Scale factor: {:.2f} mm/px".format(scale) if scale else "No scale")
                last_print_time = time.time()

            cv2.imshow("Ball Detection & Robot Navigation", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break

    except KeyboardInterrupt: 
        print("\nShutting down...")
    except Exception as e:
        print("Unexpected error: {}".format(e))
    finally:
        if cap.isOpened(): 
            cap.release()
        cv2.destroyAllWindows()
        print("Camera released and windows closed.")

if __name__ == "__main__":
    main() 