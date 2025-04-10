# Kode til hvordan robotten ved hvor den skal navigere hen (Eksempel)

import cv2
import numpy as np 
import time 
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B

# Tænd motor
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Kamera 
cap = cv2.VideoCapture(0)

def moveRobot(direction):
    if direction == "FORWARD":
        motor_a.on(50)
        motor_b.on(50)
    elif direction == "BACKWARD":
        motor_a.on(-50)
        motor_b.on(-50)
    elif direction ==  "LEFT":
        motor_a.on(30)
        motor_b.on(-30)
    elif direction == "RIGHT":
        motor_a.on(-30)
        motor_b.on(30)
    
    time.sleep(1)
    motor_a.off()
    motor_b.off

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    