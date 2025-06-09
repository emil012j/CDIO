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

<<<<<<< HEAD

=======
>>>>>>> navigation
ROBOT_IP = "172.20.10.12"
PING_PORT = 1232
COMMAND_PORT = 1233

<<<<<<< HEAD

robot_connected = False


def send_turn_command(angle_deg):
   try:
       sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
       sock.connect((ROBOT_IP, COMMAND_PORT))
       sock.send(json.dumps({"command": "turn", "angle": angle_deg}).encode())
       sock.close()
       print(f"Sendte: turn {angle_deg:.1f} degrees")
       time.sleep(abs(angle_deg) / 90)  # estimate turning time
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
       print("Ingen resultater eller OBB fundet")  ### DEBUG
       return robot_tail, robot_head, balls


   for i, box in enumerate(results.obb):
       label_index = int(box.cls)
       label = model.names[label_index].lower()
       x1, y1, x2, y2 = map(int, box.xyxy[0])
       cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

       print(f"Detected: {label} @ ({cx}, {cy})")  ### DEBUG

       
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
   return math.degrees(math.atan2(-dy, dx))


def navigate_robot_to_target(tail, head, target, path, scale, resolution):
   heading = compute_angle(tail, head)
   current_pos = snap_to_grid(tail, resolution)


   for next_pos in path:
       angle_to_next = compute_angle(current_pos, next_pos)
       turn_angle = (angle_to_next - heading + 180) % 360 - 180
       send_turn_command(turn_angle)


       distance_cm = resolution * scale / 10
       send_drive_command(distance_cm)


       heading = angle_to_next
       current_pos = next_pos


threading.Thread(target=connect_to_robot, daemon=True).start()


while not robot_connected:
   print("Venter på robotforbindelse...")
   time.sleep(1)


print("Starter kameraforbindelse og boldsøgning...")
model = YOLO("best.pt")
print("YOLO labels:", model.names)  ### DEBUG
cap = cv2.VideoCapture(0)


while True:
   ret, frame = cap.read()
   if not ret:
       break


   results_list = model(frame, imgsz=640, conf=0.5)
   if not results_list or not results_list[0]:
       print("Ingen resultater fra model")  ### DEBUG
       continue
   results = results_list[0]


   tail, head, balls = find_positions(results, model)

   print("RobotTail:", tail)
   print("RobotHead:", head)
   print("Balls:", balls)


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
           print("Valgt bold:", best_ball)  ### DEBUG
           print("Path to ball:", path)     ### DEBUG

           scale = calculate_scale_factor(results, model)
           print("Skalafaktor:", scale)     ### DEBUG

           if scale and path:
               navigate_robot_to_target(tail, head, best_ball, path, scale, resolution)
               break
           else:
               print("Ingen valid scale eller path!")  ### DEBUG


   cv2.imshow("YOLO Kamera", frame)
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

=======
robot_connected = False

def send_turn_command(angle_deg):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        sock.send(json.dumps({"command": "turn", "angle": angle_deg}).encode())
        sock.close()
        print(f"Sendte: turn {angle_deg:.1f} degrees")
        time.sleep(abs(angle_deg) / 90)  # estimate turning time
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
    return math.degrees(math.atan2(-dy, dx))

def navigate_robot_to_target(tail, head, target, path, scale, resolution):
    heading = compute_angle(tail, head)
    current_pos = snap_to_grid(tail, resolution)

    for next_pos in path:
        angle_to_next = compute_angle(current_pos, next_pos)
        turn_angle = (angle_to_next - heading + 180) % 360 - 180
        send_turn_command(turn_angle)

        distance_cm = resolution * scale / 10
        send_drive_command(distance_cm)

        heading = angle_to_next
        current_pos = next_pos

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
                navigate_robot_to_target(tail, head, best_ball, path, scale, resolution)
                break

    cv2.imshow("YOLO Kamera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
>>>>>>> navigation

cap.release()
cv2.destroyAllWindows()
