import cv2
import numpy as np
import socket
import json
import threading
import time
import heapq
from ultralytics import YOLO
from ball_identification import calculate_scale_factor
from navigation_controller import calculate_movement

ROBOT_IP = "192.168.149.158"
PING_PORT = 1232
COMMAND_PORT = 1233

robot_connected = False

def send_command_to_robot(direction, cm_distance):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"direction": direction, "distance": cm_distance}).encode())
        sock.close()
        print(f"Sendte: {direction} ({cm_distance:.1f} cm)")
        # Vent på at kommandoen bliver udført før næste iteration
        time.sleep(cm_distance / 10) 

        # Send stop command til at stoppe aktion
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"direction": "stop", "distance": 0}).encode())
        sock.close()
        print("Sendte: stop")

    except Exception as e:
        print(f"Kunne ikke sende: {e}")

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

        print(f"Fundet: {label} ved ({cx}, {cy})")

        if label == "robottail":
            robot_tail = (cx, cy)
        elif label == "robothead":
            robot_head = (cx, cy)
        elif "ball" in label and "egg" not in label:
            balls.append((cx, cy))

    return robot_tail, robot_head, balls

def build_graph_from_frame(frame, resolution=20):
    height, width, _ = frame.shape
    graph = {}
    for y in range(0, height, resolution):
        for x in range(0, width, resolution):
            node = (x, y)
            neighbors = []
            for dx, dy in [(-resolution,0), (resolution,0), (0,-resolution), (0,resolution)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    neighbors.append(((nx, ny), resolution))
            graph[node] = neighbors
    return graph

# Dijkstra's algoritme til at finde korteste veje
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    pq = [(0, start)]

    while pq:
        current_distance, current_node = heapq.heappop(pq)
        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))

    return distances

def snap_to_grid(pos, resolution):
    x, y = pos
    return (x // resolution * resolution, y // resolution * resolution)

threading.Thread(target=connect_to_robot, daemon=True).start()

while not robot_connected:
    print("Venter på robotforbindelse...")
    time.sleep(1)

print("Starter kameraforbindelse og boldsøgning...")
model = YOLO("best.pt")
print("Model labels (model.names):", model.names)
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
        resolution = 20
        graph = build_graph_from_frame(frame, resolution)
        robot_node = snap_to_grid(robot_tail, resolution)
        distances = dijkstra(graph, robot_node)

        best_ball = None
        min_distance = float('inf')
        for ball in balls:
            ball_node = snap_to_grid(ball, resolution)
            dist = distances.get(ball_node, float('inf'))
            if dist < min_distance:
                min_distance = dist
                best_ball = ball

        if best_ball:
            direction = calculate_movement(robot_tail, best_ball)
            pixel_distance = np.linalg.norm(np.array(robot_tail) - np.array(best_ball))
            scale = calculate_scale_factor(results, model)
            if scale:
                mm_distance = pixel_distance * scale
                cm_distance = mm_distance / 10
                print(f"[DIJKSTRA] Retning: {direction}, Afstand: {cm_distance:.1f} cm")
                send_command_to_robot(direction, cm_distance)
                break  # STOP efter ét move
            else:
                print("Kunne ikke udregne skala.")

    cv2.imshow("YOLO Kamera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()