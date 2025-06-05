import cv2
import numpy as np
import socket
import json
import threading
import time
import math
from ultralytics import YOLO
from ball_identification import calculate_scale_factor
from navigation_controller import calculate_movement

ROBOT_IP = "169.254.72.43"
PING_PORT = 1232
COMMAND_PORT = 1233
robot_connected = False

def send_command_to_robot(direction, cm_distance):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"direction": direction, "distance": cm_distance}).encode())
        sock.close()
        print("Sending: {} ({:.1f} cm)".format(direction, cm_distance))
        time.sleep(cm_distance / 10)  # Wait for movement to complete

        # Send stop command
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"direction": "stop", "distance": 0}).encode())
        sock.close()
        print("Sending: stop")

    except Exception as e:
        print("Could not send command: {}".format(e))

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
                        print("Received response:", ack)
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
        print("No detections from model")
        return robot_tail, robot_head, balls

    for i, box in enumerate(results.obb):
        label_index = int(box.cls)
        label = model.names[label_index].lower()
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        print("Found: {} at ({}, {})".format(label, cx, cy))

        if label == "robottail":
            robot_tail = (cx, cy)
        elif label == "robothead":
            robot_head = (cx, cy)
        elif "ball" in label and "egg" not in label:
            balls.append((cx, cy))

    return robot_tail, robot_head, balls

def compute_distance(pos1, pos2):
    """Calculate distance between two points in pixels"""
    return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

# Start robot connection ping thread
threading.Thread(target=connect_to_robot, daemon=True).start()

while not robot_connected:
    print("Waiting for robot connection...")
    time.sleep(1)

print("Starting camera and ball detection...")
model = YOLO("best.pt")
print("Model labels:", model.names)  # Print available classes
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
            
            # Calculate movement using the same function as Test_Dijkstra.py
            direction = calculate_movement(tail, closest_ball)
            pixel_distance = compute_distance(tail, closest_ball)
            distance_cm = pixel_distance * scale / 10
            
            print("Direction: {}, Distance: {:.1f} cm".format(direction, distance_cm))
            send_command_to_robot(direction, distance_cm)
            break

    # Show the frame
    cv2.imshow("YOLO Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
