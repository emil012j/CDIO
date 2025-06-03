import socket
import json
import threading
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from time import sleep

# Initialize tank drive
tank = MoveTank(OUTPUT_A, OUTPUT_D)

WHEEL_DIAMETER_CM = 7  # Updated wheel diameter

# Ports
PING_PORT = 1232
COMMAND_PORT = 1233

def cm_to_degrees(distance_cm):
    rotations = distance_cm / (3.1416 * WHEEL_DIAMETER_CM)
    return rotations * 360

# Move forward with encoder feedback
def move_robot_forward(distance_cm, speed=30):
    degrees_target = cm_to_degrees(distance_cm)

    tank.left_motor.position = 0
    tank.right_motor.position = 0

    tank.on(SpeedPercent(speed), SpeedPercent(speed))

    while True:
        left_pos = abs(tank.left_motor.position)
        right_pos = abs(tank.right_motor.position)
        if left_pos >= degrees_target and right_pos >= degrees_target:
            break
        sleep(0.01)

    tank.off()
    print("Moved forward {} cm".format(distance_cm))

# Turn by running motors for proportional time (approximate)
def move_robot_turn(angle_deg, speed=20):
    # Robot geometry
    wheel_distance_cm = 12  # Distance between wheels (axle length) — adjust if needed
    wheel_diameter_cm = WHEEL_DIAMETER_CM  # Already defined above as 7 cm

    # Compute required wheel rotations to turn by angle_deg
    robot_turn_circumference = 3.1416 * wheel_distance_cm
    rotation_fraction = abs(angle_deg) / 360
    turn_distance_cm = robot_turn_circumference * rotation_fraction
    wheel_rotations = turn_distance_cm / (3.1416 * wheel_diameter_cm)

    direction = "left" if angle_deg > 0 else "right"
    print("Turning {} {} degrees -> {:.2f} wheel rotations".format(direction, abs(angle_deg), wheel_rotations))

    if angle_deg > 0:
        tank.on_for_rotations(SpeedPercent(-speed), SpeedPercent(speed), wheel_rotations)
    else:
        tank.on_for_rotations(SpeedPercent(speed), SpeedPercent(-speed), wheel_rotations)

def handle_ping_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", PING_PORT))
    s.listen(1)
    print("EV3 Ping Server listening on port {}...".format(PING_PORT))

    while True:
        conn, addr = s.accept()
        print("Ping connection from {}".format(addr))
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                msg = json.loads(data.decode())
                if "ping" in msg:
                    conn.send(json.dumps({"ack": True}).encode())
        except Exception as e:
            print("Ping error: {}".format(e))
        finally:
            conn.close()

def execute_move_forward(distance):
    move_robot_forward(distance)

def execute_turn(angle):
    move_robot_turn(angle)

def handle_command_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", COMMAND_PORT))
    s.listen(1)
    print("EV3 Command Server listening on port {}...".format(COMMAND_PORT))

    while True:
        conn, addr = s.accept()
        print("Command connection from {}".format(addr))
        try:
            data = conn.recv(1024)
            if not data:
                conn.close()
                continue
            cmd = json.loads(data.decode())
            command = cmd.get("command")

            if command == "forward":
                distance = float(cmd.get("distance", 0))
                print("Received forward command: {} cm".format(distance))
                threading.Thread(target=execute_move_forward, args=(distance,), daemon=True).start()

            elif command == "turn":
                angle = float(cmd.get("angle", 0))
                print("Received turn command: {} degrees".format(angle))
                threading.Thread(target=execute_turn, args=(angle,), daemon=True).start()

            elif command == "stop":
                print("Received stop command: stopping motors")
                tank.off()

        except Exception as e:
            print("Command error: {}".format(e))
        finally:
            conn.close()

if __name__ == "__main__":
    threading.Thread(target=handle_ping_server, daemon=True).start()
    threading.Thread(target=handle_command_server, daemon=True).start()

    print("EV3 server running. Press Ctrl+C to stop.")
    while True:
        sleep(1)
