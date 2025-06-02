import cv2
import numpy as np
import socket
import json
import threading
import time
from ultralytics import YOLO
from ball_identification import calculate_scale_factor
from navigation_controller import calculate_movement

# IP og porte
ROBOT_IP = "192.168.149.158"
PING_PORT = 1232
COMMAND_PORT = 1233

# Global statusflag
robot_connected = False

# Sender kommando til EV3
def send_command_to_robot(direction, cm_distance):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"direction": direction, "distance": cm_distance}).encode())
        sock.close()
        print(f"Sendte: {direction} ({cm_distance:.1f} cm)")
    except Exception as e:
        print(f"Kunne ikke sende: {e}")

# Pinger robotten
def connect_to_robot():
    global robot_connected
    while True:
        try:
            print("Prøver at forbinde til robot...")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((ROBOT_IP, PING_PORT))
            print("Forbundet til robot!")

            robot_connected = True
            while True:
                try:
                    ping_message = json.dumps({'ping': True}).encode()
                    client_socket.send(ping_message)
                    response = client_socket.recv(1024)
                    if response:
                        ack = json.loads(response.decode())
                        print(" Svar modtaget:", ack)
                    time.sleep(20)
                except Exception as e:
                    print(f"Forbindelse mistet: {e}")
                    robot_connected = False
                    break
        except Exception as e:
            print(f"Forbindelse fejlede: {e}")
        time.sleep(5)


# Find objekter i billedet
def find_positions(results, model):
    

    robot_tail, robot_head = None, None
    balls = []

    if results is None or results.obb is None:
        print("Model gav ingen bokse.")
        return robot_tail, robot_head, balls
    


    for i, box in enumerate(results.obb):
        label_index = int(box.cls)
        label = model.names[label_index].lower()
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        print(f"Fundet: {label} ved ({cx}, {cy})")  # DEBUG: Print hvad modellen fandt

        if label == "robottail":
            robot_tail = (cx, cy)
        elif label == "robothead":
            robot_head = (cx, cy)
        elif "ball" in label and "egg" not in label:
            balls.append((cx, cy))

    return robot_tail, robot_head, balls

# Start forbindelse i baggrunden
threading.Thread(target=connect_to_robot, daemon=True).start()

# Vent på at robotten er forbundet før vi starter
while not robot_connected:
    print("Venter på robotforbindelse...")
    time.sleep(1)

# Når forbindelse er oppe, start kamera og YOLO
print("Starter kameraforbindelse og boldsøgning...")
model = YOLO("best.pt")
print("Model labels (model.names):", model.names)  # DEBUG: Se hvad modellen kan genkende
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kunne ikke læse fra kamera.")
        break

    results_list = model(frame, imgsz=640, conf=0.5)
    if not results_list or not results_list[0]:
        print("Ingen objekter fundet – springer billedet over.")
        continue
    results = results_list[0]

    robot_tail, robot_head, balls = find_positions(results, model)

    if robot_tail and balls:
        target = min(balls, key=lambda b: np.linalg.norm(np.array(robot_tail) - np.array(b)))
        direction = calculate_movement(robot_tail, target)

        pixel_distance = np.linalg.norm(np.array(robot_tail) - np.array(target))
        scale = calculate_scale_factor(results, model)
        if scale:
            mm_distance = pixel_distance * scale
            cm_distance = mm_distance / 10
            print(f"Retning: {direction}, Afstand: {cm_distance:.1f} cm")
            send_command_to_robot(direction, cm_distance)
        else:
            print("Kunne ikke udregne skala.")

    cv2.imshow("YOLO Kamera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
