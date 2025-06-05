import cv2
import numpy as np
import socket
import json
import threading
import time
import math
from ultralytics import YOLO
from ball_identification import calculate_scale_factor

ROBOT_IP = "169.254.72.43"
PING_PORT = 1232
COMMAND_PORT = 1233
robot_connected = False

def send_turn_command(angle_deg):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"command": "turn", "angle": angle_deg}).encode())
        sock.close()
        print("Turning {:.1f} degrees".format(angle_deg))
        # No need to sleep - the EV3 server will wait for the gyro sensor to reach the target angle
    except Exception as e:
        print("Could not send turn command: {}".format(e))

def send_drive_command(cm_distance):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"command": "forward", "distance": cm_distance}).encode())
        sock.close()
        print("Moving forward {:.1f} cm".format(cm_distance))
        time.sleep(cm_distance / 10)
    except Exception as e:
        print("Could not send drive command: {}".format(e))

def connect_to_robot():
    global robot_connected
    while True:
        try:
            print("Connecting to robot...")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((ROBOT_IP, PING_PORT))
            print("Connected to robot!")
            robot_connected = True
            while True:
                try:
                    client_socket.send(json.dumps({"ping": True}).encode())
                    response = client_socket.recv(1024)
                    if response:
                        ack = json.loads(response.decode())
                        print("Received response: {}".format(ack))
                    time.sleep(20)
                except Exception as e:
                    print("Connection lost: {}".format(e))
                    robot_connected = False
                    break
        except Exception as e:
            print("Connection failed: {}".format(e))
        time.sleep(5)

def find_positions(results, model):
    robot_tail, robot_head = None, None
    balls = []

    if results is None or results.obb is None:
        return robot_tail, robot_head, balls

    for i, box in enumerate(results.obb):
        label_index = int(box.cls)
        label = model.names[label_index].lower()
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if label == "robottail":
            robot_tail = (cx, cy)
        elif label == "robothead":
            robot_head = (cx, cy)
        elif "ball" in label and "egg" not in label:
            balls.append((cx, cy))

    return robot_tail, robot_head, balls

def compute_angle(from_pos, to_pos):
    """Calculate angle between two points in degrees"""
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return math.degrees(math.atan2(-dy, dx))  # screen coordinates: y is downward

def compute_distance(pos1, pos2):
    """Calculate distance between two points in pixels"""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

def navigate_to_ball(tail, head, ball_pos, scale):
    """
    Simple navigation: turn once to face the ball, then move straight to it.
    
    Args:
        tail: (x, y) position of robot tail
        head: (x, y) position of robot head
        ball_pos: (x, y) position of the ball
        scale: Scale factor to convert pixels to centimeters
    """
    # Calculate current robot heading (angle from tail to head)
    robot_heading = compute_angle(tail, head)
    
    # Calculate angle to ball from robot's center
    robot_center = ((head[0] + tail[0])/2, (head[1] + tail[1])/2)
    angle_to_ball = compute_angle(robot_center, ball_pos)
    
    # Calculate the turn angle needed (shortest turn)
    turn_angle = (angle_to_ball - robot_heading + 180) % 360 - 180
    
    # Calculate distance to ball in centimeters
    distance_pixels = compute_distance(robot_center, ball_pos)
    distance_cm = distance_pixels * scale / 10
    
    print("Robot heading: {:.1f} degrees".format(robot_heading))
    print("Angle to ball: {:.1f} degrees".format(angle_to_ball))
    print("Turn angle: {:.1f} degrees".format(turn_angle))
    print("Distance to ball: {:.1f} cm".format(distance_cm))
    
    # Execute the movement
    # First turn to face the ball
    send_turn_command(turn_angle)
    
    # Then move straight to the ball
    send_drive_command(distance_cm)

# Start robot connection ping thread
threading.Thread(target=connect_to_robot, daemon=True).start()

while not robot_connected:
    print("Waiting for robot connection...")
    time.sleep(1)

print("Starting camera and ball detection...")
model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Could not read from camera")
        break

    # Convert BGR to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Run YOLO detection
    results_list = model([frame_rgb], imgsz=640, conf=0.5)
    
    if not results_list or not results_list[0].boxes:
        continue

    results = results_list[0]
    tail, head, balls = find_positions(results, model)

    if tail and head and balls:
        # Find closest ball
        robot_center = ((head[0] + tail[0])/2, (head[1] + tail[1])/2)
        closest_ball = min(balls, key=lambda b: compute_distance(robot_center, b))
        
        # Calculate scale factor
        scale = calculate_scale_factor(results, model)
        
        if scale:
            # Draw visualization
            cv2.line(frame, (int(robot_center[0]), int(robot_center[1])), 
                    (int(closest_ball[0]), int(closest_ball[1])), (0, 255, 0), 2)
            cv2.circle(frame, (int(closest_ball[0]), int(closest_ball[1])), 5, (0, 0, 255), -1)
            
            # Navigate to the ball
            navigate_to_ball(tail, head, closest_ball, scale)
            break

    # Show the frame
    cv2.imshow("YOLO Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
