# -*- coding: utf-8 -*-
"""
Indstillinger for robotten, feks. IP, port, kamera, modeller, farver, etc.
"""

# Netværk indstillinger (IP, port)
ROBOT_IP = "169.254.71.123"
COMMAND_PORT = 1233

# Kamera indstillinger (resolution, confidence)
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (1280, 720)
CONFIDENCE_THRESHOLD = 0.35

# YOLO model indstillinger
MODEL_PATH = "best.pt"

# Object detection konstanter
BALL_DIAMETER_MM = 40
CROSS_DIAMETER_MM = 200.0
EGG_SIZE_THRESHOLD_MM = 58.0

# Farver til visualization
CLASS_COLORS = {
    "cross": (0, 255, 0), 
    "egg": (255, 0, 0), 
    "orange ball": (0, 140, 255), 
    "white ball": (255, 255, 255), 
    "robothead": (0, 0, 255), 
    "robottail": (255, 0, 255)
}

# Vision system indstillinger
ORIENTED_OBJECTS = ["robothead", "robottail", "egg", "cross"]
EGG_OBB_ASPECT_RATIO_THRESHOLD = 1.3

# Navigation parametre (turn threshold, distances)
COMMAND_COOLDOWN = 0.5  # sekunder mellem kommandoer, feks. hvis robotten ikke er klar til at modtage kommandoer
PRINT_INTERVAL = 3      # print status hver 3 sekunder. status er feks. hvor robotten er, hvor bolden er, hvor egget er, etc.
TURN_THRESHOLD = 10     # grader - robotten drejer kun hvis fejl er større end 10°.
DISTANCE_THRESHOLD = 5  # cm - robotten kører kun hvis afstand er større end 5cm
MAX_TURN_INCREMENT = 30 # drejer maksimum 30 grader ad gangen når den sigter mod boldene. ved at øge det kan det evt blive mere clean når den dreje mod boldene.
MAX_FORWARD_DISTANCE = 20 # maksimum 20cm ad gangen 

# Robot hardware værdier (hjul diameter, hastigheder)
WHEEL_DIAMETER_CM = 7.0
ROBOT_TURN_SPEED = 40 #hastigheden på robotten, når den drejer
ROBOT_FORWARD_SPEED = 50 #hastigheden på robotten, når den kører fremad
ESTIMATED_TURN_RATE = 180.0  # grader per sekund
ESTIMATED_FORWARD_RATE = 20.0  # cm per sekund, 

# PID Settings (vi bruger det ikke gyro indtil videre - kun hvis vi vil bruge gyro til præcis navigation)
PID_KP = 0.5   # Proportional - hvor hurtigt robotten reagerer på fejl (højere = mere aggressiv)
PID_KI = 0.01  # Integral - husker gamle fejl og korrigerer langsomt (forhindrer permanent fejl)
PID_KD = 0.3   # Derivative - forudser fremtidige fejl (forhindrer oversving)