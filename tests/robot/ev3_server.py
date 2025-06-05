import socket
import json
import threading
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1  # Assuming gyro is in port 1
from ev3dev2.sensor.lego import GyroSensor
from time import sleep
import math

# Initialize tank drive and gyro sensor
tank = MoveTank(OUTPUT_A, OUTPUT_D)
gyro = GyroSensor(INPUT_1)
gyro.mode = 'GYRO-ANG'  # Set to angle mode

# Robot physical parameters
WHEEL_DIAMETER_CM = 7.62  # 3.81 * 2 (radius to diameter)
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM
ROBOT_WIDTH_CM = 15  # Distance between wheels
TURN_CIRCUMFERENCE_CM = math.pi * ROBOT_WIDTH_CM  # Circumference of the circle the robot makes when turning

# Ports
PING_PORT = 1232
COMMAND_PORT = 1233

def reset_gyro():
    """Reset the gyro sensor to 0 degrees"""
    gyro.mode = 'GYRO-RATE'  # Switch to rate mode
    sleep(0.1)  # Wait for sensor to stabilize
    gyro.mode = 'GYRO-ANG'   # Switch back to angle mode
    sleep(0.1)  # Wait for sensor to stabilize

def get_current_angle():
    """Get the current angle from the gyro sensor"""
    return gyro.angle

def cm_to_degrees(distance_cm):
    """Convert centimeters to motor degrees"""
    rotations = distance_cm / WHEEL_CIRCUMFERENCE_CM
    return rotations * 360

def degrees_to_cm(degrees):
    """Convert motor degrees to centimeters"""
    rotations = degrees / 360
    return rotations * WHEEL_CIRCUMFERENCE_CM

def calculate_turn_degrees(angle_deg):
    """
    Calculate how many degrees the motors need to turn to rotate the robot by angle_deg.
    This is based on the robot's wheel distance and wheel circumference.
    
    Args:
        angle_deg: Desired rotation angle in degrees (positive = right, negative = left)
    
    Returns:
        float: Number of degrees the motors need to turn
    """
    # Calculate the distance the outer wheel needs to travel
    # For a full 360-degree turn, the outer wheel travels the circumference of the turn circle
    turn_distance = (abs(angle_deg) / 360) * TURN_CIRCUMFERENCE_CM
    
    # Convert this distance to motor degrees
    motor_degrees = cm_to_degrees(turn_distance)
    
    return motor_degrees

def move_robot_forward(distance_cm, speed=30):
    """Move the robot forward a specific distance using encoders"""
    degrees_target = cm_to_degrees(distance_cm)
    
    # Reset motor positions
    tank.left_motor.position = 0
    tank.right_motor.position = 0
    
    # Start moving
    tank.on(SpeedPercent(speed), SpeedPercent(speed))
    
    # Wait until both motors reach target
    while True:
        left_pos = abs(tank.left_motor.position)
        right_pos = abs(tank.right_motor.position)
        if left_pos >= degrees_target and right_pos >= degrees_target:
            break
        sleep(0.01)
    
    tank.off()
    print(f"Moved forward {distance_cm:.1f} cm")

def move_robot_turn(target_angle_deg, speed=20):
    """
    Turn the robot to a specific angle using the gyro sensor.
    Positive angle = right turn, negative angle = left turn.
    """
    # Reset gyro to 0
    reset_gyro()
    
    # Determine turn direction and set motor speeds
    if target_angle_deg > 0:  # Right turn
        tank.on(SpeedPercent(-speed), SpeedPercent(speed))
    else:  # Left turn
        tank.on(SpeedPercent(speed), SpeedPercent(-speed))
    
    # Wait until we reach the target angle
    while True:
        current_angle = get_current_angle()
        if abs(current_angle) >= abs(target_angle_deg):
            break
        sleep(0.01)
    
    # Stop the motors
    tank.off()
    
    # Small correction if needed
    current_angle = get_current_angle()
    angle_error = target_angle_deg - current_angle
    if abs(angle_error) > 2:  # If error is more than 2 degrees
        correction_speed = 10  # Slower speed for correction
        if angle_error > 0:
            tank.on(SpeedPercent(-correction_speed), SpeedPercent(correction_speed))
        else:
            tank.on(SpeedPercent(correction_speed), SpeedPercent(-correction_speed))
        
        while abs(get_current_angle() - target_angle_deg) > 1:
            sleep(0.01)
        tank.off()
    
    print(f"Turned to {get_current_angle():.1f} degrees (target was {target_angle_deg:.1f})")

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
