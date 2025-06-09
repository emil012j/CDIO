# -*- coding: utf-8 -*-
"""
Indstillinger for robotten, feks. IP, port, kamera, modeller, farver, etc.
"""

# Netværk indstillinger (IP, port)
ROBOT_IP = "169.254.200.88"
COMMAND_PORT = 1233

# Kamera indstillinger (resolution, confidence) - OPTIMERET FOR PERFORMANCE
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (640, 480)  # Reduceret fra 1280x720 - STOR performance forbedring!
CONFIDENCE_THRESHOLD = 0.25  # Øget lidt for færre false positives

# Performance indstillinger - NYE OPTIMERINGSPARAMETRE
TARGET_FPS = 15  # Begrænser FPS for stabil performance
YOLO_FRAME_SKIP = 3  # Kør kun YOLO på hver 3. frame - KÆMPE performance boost!
YOLO_IMG_SIZE = 416  # Reduceret fra 640 - hurtigere inference
ENABLE_THREADING = True  # Kør kamera og processing i separate threads

# YOLO model indstillinger
MODEL_PATH = "best.pt"

# Object detection konstanter  
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
TURN_THRESHOLD = 10     # mindste antal grader for at dreje.
DISTANCE_THRESHOLD = 3  # cm - kør helt tæt på for at samle boldene
MAX_FORWARD_DISTANCE = 20 # maks længde for fremadkørsel

ROBOT_TURN_SPEED = 40 # Motor hastighed til drejning
ROBOT_FORWARD_SPEED = 50 # Motor hastighed til fremadkørsel  
ESTIMATED_TURN_RATE = 120.0  # grader per sekund - til konvertering af duration til vinkel