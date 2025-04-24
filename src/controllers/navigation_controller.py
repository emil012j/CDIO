import cv2
import numpy as np 
import time 
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ultralytics import YOLO

# Tænd motor
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Kamera
cap = cv2.VideoCapture(0)

# YOLO model
model = YOLO("my_model2.pt")  # Indlæs din model

# Udregn afstanden
def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

# Udregn hvor robotten skal bevæge sig
def calculate_movement(robot_pos, target_pos, avoid=False):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]

    if avoid:  # Hvis vi vil undgå ægget, ændrer vi retningen
        dx = -dx
        dy = -dy

    if abs(dx) > abs(dy):
        return "RIGHT" if dx > 0 else "LEFT"
    else:
        return "FORWARD" if dy > 0 else "BACKWARD"

def moveRobot(direction):
    if direction == "FORWARD":
        motor_a.on(50)
        motor_b.on(50)
    elif direction == "BACKWARD":
        motor_a.on(-50)
        motor_b.on(-50)
    elif direction ==  "LEFT":
        motor_a.on(30)
        motor_b.on(-30)
    elif direction == "RIGHT":
        motor_a.on(-30)
        motor_b.on(30)
    
    time.sleep(1)
    motor_a.off()
    motor_b.off()

# Robotens nuværende position (kan f.eks. være midt i billedet)
robot_pos = (640, 360)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, imgsz=640, conf=0.50)[0]

    target_pos = None
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = model.names[int(box.cls)]
        if label == "egg":
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            target_pos = (cx, cy)
            break

    if target_pos is not None:
        print("Æg detekteret. Undgår ægget.")
        direction = calculate_movement(robot_pos, target_pos, avoid=True)
    else:
        print("Ingen æg fundet. Navigerer normalt.")
        direction = calculate_movement(robot_pos, (640, 360))

    print(f"Bevægelse mod: {direction}")
    moveRobot(direction)

    # Vis billedet med bokse og labels
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = model.names[int(box.cls)]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    cv2.imshow("Webcam feed", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()