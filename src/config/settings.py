# -*- coding: utf-8 -*-
"""
Indstillinger for robotten, feks. IP, port, kamera, modeller, farver, etc.
"""

# Netværk indstillinger (IP, port)
ROBOT_IP = "192.168.8.97"
COMMAND_PORT = 1233

# Kamera indstillinger (resolution, confidence)
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (1280, 720)  # REDUCED from 1280x720 for better performance
CONFIDENCE_THRESHOLD = 0.35 #0.35

# YOLO model indstillinger
MODEL_PATH = "my_model3.pt"

# Object detection konstanter  
CROSS_DIAMETER_MM = 200.0
EGG_SIZE_THRESHOLD_MM = 58.0
CROSS_AVOID_RADIUS = 100  # Tune as needed. Bliver brugt til at undgå at robotten kører ind i krydset.

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
COMMAND_COOLDOWN = 2.0  # Increased to prevent network spam that causes lag
PRINT_INTERVAL = 5      # 5 sekunder som i den gamle fil
TURN_THRESHOLD = 10     # mindste antal grader for at dreje.
DISTANCE_THRESHOLD = 30 # cm - stop 30 cm fra bolden for præcis korrektion
MAX_FORWARD_DISTANCE = 30 # maks længde for fremadkørsel

# Ball pickup sekvens - hardcoded afstande
PICKUP_FORWARD_DISTANCE = 35  # cm - afstand at køre frem for at samle bold
PICKUP_BACKWARD_DISTANCE = 35  # cm - afstand at bakke efter opsamling

ROBOT_TURN_SPEED = 10 # Motor hastighed til drejning
ROBOT_FORWARD_SPEED = 30 # Motor hastighed til fremadkørsel  
ESTIMATED_TURN_RATE = 180.0  # grader per sekund - opdateret baseret på hjul kalibrering (halv rotation = 90°)