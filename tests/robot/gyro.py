#!/usr/bin/env python3
import socket
import json
import time

try:
    from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
    from ev3dev2.sensor import INPUT_1
    from ev3dev2.sensor.lego import GyroSensor
except ImportError:
  
    LargeMotor = None
    OUTPUT_B = None
    OUTPUT_C = None
    SpeedPercent = None
    INPUT_1 = None
    GyroSensor = None

ROBOT_IP = "192.168.62.158"  # ← skift IP hvis nødvendigt
COMMAND_PORT = 1233

def send_turn_command(angle_deg):
    try:
        print(f"Sender turn {angle_deg}° til robotten via netværk...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        command = {"command": "turn", "angle": angle_deg}
        sock.send(json.dumps(command).encode())
        sock.close()
        print("Kommando sendt.")
    except Exception as e:
        print(f"Fejl ved sending: {e}")



if LargeMotor and GyroSensor:
    gyro  = GyroSensor(INPUT_1)
    left  = LargeMotor(OUTPUT_B)
    right = LargeMotor(OUTPUT_C)

    TURN_SPEED = 20
    FWD_SPEED = 30
    WHEEL_CIRC = 17.6
    TICKS_PER_CM = 360 / WHEEL_CIRC

    def gyro_turn(angle_deg):
        gyro.reset()
        direction = 1 if angle_deg > 0 else -1
        left.run_forever(speed_sp = -TURN_SPEED * direction)
        right.run_forever(speed_sp = TURN_SPEED * direction)

        while abs(gyro.angle) < abs(angle_deg) - 1:
            time.sleep(0.01)

        left.stop()
        right.stop()
        print(f"Drejet {gyro.angle:.1f}°")

    def drive_forward(cm):
        ticks = int(cm * TICKS_PER_CM)
        left.on_for_degrees(SpeedPercent(FWD_SPEED), ticks, block=False)
        right.on_for_degrees(SpeedPercent(FWD_SPEED), ticks)
else:
    def gyro_turn(angle_deg):
        print("Gyro-funktion ikke tilgængelig (ikke på EV3)")

    def drive_forward(cm):
        print("Drive forward ikke tilgængelig (ikke på EV3)")


if __name__ == "__main__":


    brug_netvaerk = False   

    if brug_netvaerk:
        send_turn_command(90)
        time.sleep(5)
        print("Netværks-test færdig.")
    else:
        print("Starter lokal gyro-drej …")
        gyro_turn(90)
        time.sleep(0.5)
        print("Kører 30 cm frem …")
        drive_forward(30)
        print("Lokal test færdig.")
