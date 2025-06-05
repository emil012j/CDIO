import cv2
import numpy as np
import socket
import json
import threading
import time
import heapq
import math
from ultralytics import YOLO
from ball_identification import calculate_scale_factor

ROBOT_IP = "192.168.62.158"
PING_PORT = 1232
COMMAND_PORT = 1233
robot_connected = False

def send_turn_command(angle_deg):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"command": "turn", "angle": angle_deg}).encode())
        sock.close()
        print(f"Sendte: turn {angle_deg:.1f} degrees")
        time.sleep(abs(angle_deg) / 90)  # Estimate time based on angle
    except Exception as e:
        print(f"Kunne ikke sende turn: {e}")

def send_drive_command(cm_distance):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"command": "forward", "distance": cm_distance}).encode())
        sock.close()
        print(f"Sendte: forward {cm_distance:.1f} cm")
        time.sleep(cm_distance / 10)
    except Exception as e:
        print(f"Kunne ikke sende drive: {e}")

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
                    client_socket.send(json.dumps({'ping': True}).encode())
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

    # Sanity check: er de for tæt på hinanden?
    if robot_tail and robot_head:
        dist = math.hypot(robot_head[0] - robot_tail[0], robot_head[1] - robot_tail[1])
        if dist < 10:
            print("Robothead og robottail er for tæt på hinanden – springer frame over")
            return None, None, balls

    return robot_tail, robot_head, balls

def build_graph_from_frame(frame, resolution=20):
    height, width, _ = frame.shape
    graph = {}
    for y in range(0, height, resolution):
        for x in range(0, width, resolution):
            node = (x, y)
            neighbors = []
            for dx, dy in [(-resolution, 0), (resolution, 0), (0, -resolution), (0, resolution)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    neighbors.append(((nx, ny), resolution))
            graph[node] = neighbors
    return graph

def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    previous = {node: None for node in graph}
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
                previous[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    return distances, previous

def reconstruct_path(previous, start, goal):
    path = []
    current = goal
    while current and current != start:
        path.append(current)
        current = previous[current]
    path.reverse()
    return path

def snap_to_grid(pos, resolution):
    x, y = pos
    return (x // resolution * resolution, y // resolution * resolution)

def compute_angle(from_pos, to_pos):
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return math.degrees(math.atan2(-dy, dx))  # screen coordinates: y is downward

def navigate_robot_to_target(tail, head, target, path, scale, resolution, model, cap):
    heading = compute_angle(tail, head)
    current_pos = snap_to_grid(tail, resolution)
    segments = compress_path(path)

    for start, end in segments:
        angle_to_next = compute_angle(current_pos, end)
        turn_angle = (angle_to_next - heading + 180) % 360 - 180

        if abs(turn_angle) > 5:
            send_turn_command(turn_angle)

        pixel_distance = np.linalg.norm(np.array(end) - np.array(current_pos))
        cm_distance = (pixel_distance * scale) / 10
        if cm_distance > 2:
            send_drive_command(cm_distance)

        # 🔁 Tag nyt billede og opdater heading og position
        ret, frame = cap.read()
        if not ret:
            print("Kamerafejl – afbryder navigation")
            break
        results_list = model(frame, imgsz=640, conf=0.5)
        if not results_list or not results_list[0]:
            print("Ingen resultater efter bevægelse")
            break
        results = results_list[0]
        tail, head, _ = find_positions(results, model)
        if not tail or not head:
            print("Mangler robothead eller tail – stopper navigation")
            break

        heading = compute_angle(tail, head)
        current_pos = snap_to_grid(tail, resolution)


def compress_path(path):
    if not path:
        return []

    compressed = []
    prev = path[0]
    direction = None
    group = [prev]

    for curr in path[1:]:
        dx = curr[0] - prev[0]
        dy = curr[1] - prev[1]
        if dx == 0 and dy == 0:
            continue
        new_direction = math.degrees(math.atan2(-dy, dx))
        if direction is None or abs(new_direction - direction) < 1e-3:
            group.append(curr)
        else:
            compressed.append((group[0], group[-1]))
            group = [prev, curr]
        direction = new_direction
        prev = curr

    if group:
        compressed.append((group[0], group[-1]))

    return compressed


# Start robot connection ping thread
threading.Thread(target=connect_to_robot, daemon=True).start()

while not robot_connected:
    print("Venter på robotforbindelse...")
    time.sleep(1)

print("Starter kameraforbindelse og boldsøgning...")
model = YOLO("best.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results_list = model(frame, imgsz=640, conf=0.5)
    if not results_list or not results_list[0]:
        continue

    results = results_list[0]
    tail, head, balls = find_positions(results, model)

    if tail and head and balls:
        resolution = 20
        graph = build_graph_from_frame(frame, resolution)
        robot_node = snap_to_grid(tail, resolution)
        distances, previous = dijkstra(graph, robot_node)

        best_ball = None
        min_distance = float('inf')
        for ball in balls:
            ball_node = snap_to_grid(ball, resolution)
            dist = distances.get(ball_node, float('inf'))
            if dist < min_distance:
                min_distance = dist
                best_ball = ball

        if best_ball:
            ball_node = snap_to_grid(best_ball, resolution)
            path = reconstruct_path(previous, robot_node, ball_node)
            scale = calculate_scale_factor(results, model)

            if scale and path:
                navigate_robot_to_target(tail, head, best_ball, path, scale, resolution, model, cap)
                break

    cv2.imshow("YOLO Kamera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
