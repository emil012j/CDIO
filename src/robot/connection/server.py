#!/usr/bin/env python3
import socket
import json
import threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from time import sleep

# Motorer
motor_a = LargeMotor(OUTPUT_A)
motor_d = LargeMotor(OUTPUT_D)

# PORTINDDELING
PORT = 1232
COMMAND_PORT = 1233

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

    if direction == "FORWARD":
        motor_a.on(50)
        motor_d.on(50)
    elif direction == "BACKWARD":
        motor_a.on(-50)
        motor_d.on(-50)
    elif direction == "LEFT":
        motor_a.on(-30)
        motor_d.on(30)
    elif direction == "RIGHT":
        motor_a.on(30)
        motor_d.on(-30)

    sleep(duration)
    motor_a.off()
    motor_d.off()

# Start begge servere i parallelle traade
if __name__ == "__main__":
    threading.Thread(target=handle_ping_server, daemon=True).start()
    threading.Thread(target=handle_command_server, daemon=True).start()

    print("EV3-server koerer. Tryk Ctrl+C for at stoppe.")

    # Hold main thread i live
    while True:
        sleep(1)
