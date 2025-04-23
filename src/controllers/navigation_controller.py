# Kode til hvordan robotten ved hvor den skal navigere hen (Eksempel)

import cv2
import numpy as np 
import time 
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B

# Tænd motor
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Kamera 
cap = cv2.VideoCapture(0)

# Udregn afstanden
def calculate_distance(pos1, pos2):
    return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos2[1] ** 2))

# Udregn hvor robotten skal bevæge sig 
def calculate_movement(robot_pos, target_pos):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]

    if abs(dx) > abs(dy):
        return "RIGHT" if dx > 0 else "LEFT"
    else:
        return "FORWARD" if dy > 0 else "BACKWARD"


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

# Definer robotten og måls positioner



while True:
    ret, frame = cap.read()
    if not ret:
        break






    