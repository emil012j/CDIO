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

    if avoid:
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
    elif direction == "LEFT":
        motor_a.on(30)
        motor_b.on(-30)
    elif direction == "RIGHT":
        motor_a.on(-30)
        motor_b.on(30)

    time.sleep(1)
    motor_a.off()
    motor_b.off()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, imgsz=640, conf=0.5)[0]

    robot_head = None
    robot_tail = None
    balls = []
    eggs = []

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
        print(f"🤖 Robot position: {robot_pos}")

        # Find nærmeste æg
        if eggs:
            closest_egg = min(eggs, key=lambda e: calculate_distance(robot_pos, e))
            print("🥚 Æg fundet, undgår...")
            direction = calculate_movement(robot_pos, closest_egg, avoid=True)
        elif balls:
            target_pos = min(balls, key=lambda b: calculate_distance(robot_pos, b))
            print(f"⚽ Går mod bold: {target_pos}")
            direction = calculate_movement(robot_pos, target_pos)
        else:
            print("🚫 Ingen mål fundet. Står stille.")
            continue  # skip moving if nothing found

        print(f"➡️ Retning: {direction}")
        moveRobot(direction)

    else:
        print("⚠️ Robotens position kunne ikke bestemmes.")

    cv2.imshow("Webcam feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()