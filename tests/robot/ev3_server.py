#!/usr/bin/env python3
# bruges ikke - gammel test fil
import socket
import json
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
from time import sleep
import time

# Motorer
motor_a = LargeMotor(OUTPUT_A)
motor_d = LargeMotor(OUTPUT_D)

# Sensorer
gyro = GyroSensor(INPUT_1)
gyro.mode = 'GYRO-ANG'  # Sæt mode til vinkel-måling
gyro.reset()  # Nulstil gyro

# PORTINDDELING
PORT = 1232
COMMAND_PORT = 1233

# Konstanter til PID-controller
KP = 2  # Proportional gain
KI = 0  # Integral gain
KD = 0  # Derivative gain
BASE_SPEED = 30  # Basis hastighed

def pid_straight(target_angle, speed):
    """Kør lige ud med PID-kontrol baseret på gyro"""
    error = gyro.angle - target_angle
    correction = error * KP
    
    left_speed = speed - correction
    right_speed = speed + correction
    
    # Begræns hastigheder
    left_speed = max(-100, min(100, left_speed))
    right_speed = max(-100, min(100, right_speed))
    
    return left_speed, right_speed

# 🔁 Ping-server
def handle_ping_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", PORT))
    s.listen(1)
    print("EV3 Server listening on port {}...".format(PORT))

    while True:
        conn, addr = s.accept()
        print("Ping-forbindelse fra {}".format(addr))
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    print("Ping-afbrudt.")
                    break
                msg = json.loads(data.decode())
                if "ping" in msg:
                    conn.send(json.dumps({"ack": True}).encode())
        except Exception as e:
            print("Ping-fejl:", e)
        finally:
            conn.close()

# Kommandoserver
def handle_command_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", COMMAND_PORT))
    s.listen(1)
    print("Kommando-server lytter paa port {}".format(COMMAND_PORT))

    while True:
        conn, addr = s.accept()
        print("Kommando modtaget fra {}".format(addr))
        try:
            data = conn.recv(1024)
            if not data:
                continue
            cmd = json.loads(data.decode())
            direction = cmd.get("direction")
            distance = float(cmd.get("distance", 0))
            move_robot(direction, distance)
        except Exception as e:
            print("Fejl i kommando:", e)
        finally:
            conn.close()

# Bevaegelsesfunktion
def move_robot(direction, distance_cm):
    speed = 10  # cm/s
    duration = distance_cm / speed
    print("koerer {} i {} sekunder ({} cm)".format(direction, duration, distance_cm))

    # Gem start vinkel
    start_angle = gyro.angle

    if direction == "FORWARD":
        # Kør fremad med PID-kontrol
        end_time = time.time() + duration
        while time.time() < end_time:
            left_speed, right_speed = pid_straight(start_angle, -BASE_SPEED)  # Negativ for fremad
            motor_a.on(left_speed)
            motor_d.on(right_speed)
            sleep(0.01)  # Kort pause mellem justeringer
    elif direction == "BACKWARD":
        # Kør baglæns med PID-kontrol
        end_time = time.time() + duration
        while time.time() < end_time:
            left_speed, right_speed = pid_straight(start_angle, BASE_SPEED)  # Positiv for baglæns
            motor_a.on(left_speed)
            motor_d.on(right_speed)
            sleep(0.01)  # Kort pause mellem justeringer
    elif direction == "LEFT":
        motor_a.on(30)
        motor_d.on(-30)
        sleep(duration)
    elif direction == "RIGHT":
        motor_a.on(-30)
        motor_d.on(30)
        sleep(duration)

    motor_a.off()
    motor_d.off()

# Start begge servere i parallelle traade
if __name__ == "__main__":
    # Kalibrer gyro ved start
    print("Kalibrerer gyro...")
    gyro.calibrate()
    sleep(2)  # Vent på kalibrering er færdig
    print("Gyro kalibreret. Start vinkel:", gyro.angle)

    threading.Thread(target=handle_ping_server, daemon=True).start()
    threading.Thread(target=handle_command_server, daemon=True).start()

    print("EV3-server koerer. Tryk Ctrl+C for at stoppe.")

    # Hold main thread i live
    while True:
        sleep(1)
