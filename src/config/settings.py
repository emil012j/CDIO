# -*- coding: utf-8 -*-
"""
Indstillinger for robotten, feks. IP, port, kamera, modeller, farver, etc.
"""

# Netværk indstillinger (IP, port)
ROBOT_IP = "169.254.6.188"
COMMAND_PORT = 1233

# Kamera indstillinger (resolution, confidence)
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (1280, 720)
CONFIDENCE_THRESHOLD = 0.35 #0.35

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

# Navigation parametre - ingen sensorer, kun vision navigation
COMMAND_COOLDOWN = 1.0  # Fra gamle ball_identification.py
PRINT_INTERVAL = 5      # 5 sekunder som i den gamle fil
TURN_THRESHOLD = 15     # grader - større threshold for ordentlig rotation først
DISTANCE_THRESHOLD = 3  # cm - kør helt tæt på for at samle boldene
MAX_TURN_INCREMENT = 45 # Fra gamle fil - større drejninger
MAX_FORWARD_DISTANCE = 20 # længere fremad for at nå boldene 

# Robot hardware værdier (hjul diameter, hastigheder) - optimeret for tydelige bevægelser
WHEEL_DIAMETER_CM = 7.0
ROBOT_TURN_SPEED = 40 # Fra gamle robot controller
ROBOT_FORWARD_SPEED = 50 # Fra gamle robot controller  
ESTIMATED_TURN_RATE = 120.0  # samme som før
ESTIMATED_FORWARD_RATE = 20.0  # Fra gamle fil (distance/20.0) 

# PID Settings (vi bruger det ikke gyro indtil videre - kun hvis vi vil bruge gyro til præcis navigation)
PID_KP = 0.5   # Proportional - hvor hurtigt robotten reagerer på fejl (højere = mere aggressiv)
PID_KI = 0.01  # Integral - husker gamle fejl og korrigerer langsomt (forhindrer permanent fejl)
PID_KD = 0.3   # Derivative - forudser fremtidige fejl (forhindrer oversving)