import cv2
import numpy as np 
import time 
import math
import socket
import json
from ultralytics import YOLO

# Netværks konfiguration
# For at finde EV3's IP:
# 1. På EV3: Gå til Wireless and Networks -> All Network Connections
# 2. Vælg din forbindelse og find IP adressen
ROBOT_IP = "192.168.149.158"  # ← Opdater denne med EV3's IP
COMMAND_PORT = 1233

def send_coordinate_command(target_pos, current_pos, scale_factor):
    """Send koordinater til robotten"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        
        # Konverter pixel koordinater til cm
        target_x_cm = target_pos[0] * scale_factor
        target_y_cm = target_pos[1] * scale_factor
        current_x_cm = current_pos[0] * scale_factor
        current_y_cm = current_pos[1] * scale_factor
        
        command = {
            "coordinates": {
                "target_x": target_x_cm,
                "target_y": target_y_cm,
                "current_x": current_x_cm,
                "current_y": current_y_cm
            }
        }
        
        sock.send(json.dumps(command).encode())
        print(f"Sendte koordinater: Mål ({target_x_cm:.1f}, {target_y_cm:.1f}) cm, "
              f"Nuværende ({current_x_cm:.1f}, {current_y_cm:.1f}) cm")
        sock.close()
        
    except Exception as e:
        print(f"Kunne ikke sende koordinater: {e}")

<<<<<<< HEAD
# Kamera
cap = cv2.VideoCapture(0)

# YOLO model
model = YOLO("best.pt")  # Indlæs din model

# Udregn afstanden
=======
>>>>>>> navigation
def calculate_distance(pos1, pos2):
    """Beregn afstand mellem to punkter"""
    return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

def find_robot_and_balls(frame, model):
    """Find robot og bold positioner"""
    results = model(frame, imgsz=640, conf=0.5)[0]
    
    robot_head = None
    robot_tail = None
    balls = []
    
    if results and results.boxes:
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls)]
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            if label == "robothead":
                robot_head = (cx, cy)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            elif label == "robottail":
                robot_tail = (cx, cy)
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            elif "ball" in label.lower() and "egg" not in label.lower():
                balls.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    
    return robot_head, robot_tail, balls, frame

def main():
    # Start kamera
    cap = cv2.VideoCapture(0)
    model = YOLO("best.pt")
    
    # Konstant skaleringsfaktor (pixel til cm) - juster efter behov
    SCALE_FACTOR = 0.1  # Dette skal kalibreres!
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kunne ikke læse fra kamera")
            break
            
        # Find objekter
        robot_head, robot_tail, balls, frame = find_robot_and_balls(frame, model)
        
        # Hvis vi har både robot position og bolde
        if robot_head and robot_tail and balls:
            # Beregn robot centrum
            robot_pos = (
                (robot_head[0] + robot_tail[0]) // 2,
                (robot_head[1] + robot_tail[1]) // 2
            )
            
            # Find nærmeste bold
            closest_ball = min(balls, key=lambda b: calculate_distance(robot_pos, b))
            
            # Send koordinater til robot
            send_coordinate_command(closest_ball, robot_pos, SCALE_FACTOR)
            
            # Tegn linjer på frame
            cv2.line(frame, robot_pos, closest_ball, (0, 255, 255), 2)
            
        # Vis frame
        cv2.imshow("Navigation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

<<<<<<< HEAD
    results = results_list[0]

    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = model.names[int(box.cls)]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        if label == "robothead":
            robot_head = (cx, cy)
        elif label == "robottail":
            robot_tail = (cx, cy)
        elif label == "white ball":
            balls.append((cx, cy))
        elif label == "egg":
            eggs.append((cx, cy))

        # Vis boks og label på billedet
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # Beregn robot position
    if robot_head and robot_tail:
        robot_pos = (
            (robot_head[0] + robot_tail[0]) // 2,
            (robot_head[1] + robot_tail[1]) // 2
        )
        print(f"Robot position: {robot_pos}")

        # Find nærmeste æg
        if eggs:
            closest_egg = min(eggs, key=lambda e: calculate_distance(robot_pos, e))
            print("Æg fundet, undgår...")
            direction = calculate_movement(robot_pos, closest_egg, avoid=True)
        elif balls:
            target_pos = min(balls, key=lambda b: calculate_distance(robot_pos, b))
            print(f"Går mod bold: {target_pos}")
            direction = calculate_movement(robot_pos, target_pos)
        else:
            print("Ingen mål fundet. Står stille.")
            continue  # skip moving if nothing found

        print(f"Retning: {direction}")

    else:
        print("Robotens position kunne ikke bestemmes.")

    cv2.imshow("Webcam feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

"""
    robot_dx = robot_head[0] - robot_tail[0]
    robot_dy = robot_head[1] - robot_tail[1]
    robot_angle = math.atan2(robot_dy, robot_dx)

    # Vector from tail to target
    target_dx = best_ball[0] - robot_tail[0]
    target_dy = best_ball[1] - robot_tail[1]
    target_angle = math.atan2(target_dy, target_dx)

    # Calculate angle difference in degrees
    angle_diff = math.degrees(target_angle - robot_angle)
    angle_diff = (angle_diff + 360) % 360  # Normalize to [0, 360)

    if angle_diff < 45 or angle_diff > 315:
        return "FORWARD"
    elif 45 <= angle_diff < 135:
        return "RIGHT"
    elif 135 <= angle_diff < 225:
        return "BACKWARD"
    else:
        return "LEFT"

"""
=======
if __name__ == "__main__":
    main()
>>>>>>> navigation
