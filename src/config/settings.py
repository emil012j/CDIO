# -*- coding: utf-8 -*-
"""
Settings for the robot, e.g. IP, port, camera, models, colors, etc.
"""

# Network settings (IP, port)
ROBOT_IP = "192.168.158.158"
COMMAND_PORT = 1233

# Camera settings (resolution, confidence)
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (1280, 720)  
CONFIDENCE_THRESHOLD = 0.35 #0.35

# YOLO model settings
MODEL_PATH = "my_model3.pt"

# Object detection constants  
CROSS_DIAMETER_MM = 200.0
EGG_SIZE_THRESHOLD_MM = 58.0
CROSS_AVOID_RADIUS = 100  # Tune as needed. Used to avoid the robot running into the cross.
GOAL_LEFT = (0, 300) # Left goal position in pixels
GOAL_RIGHT = (1279, 300) # Right goal position in pixels

# Colors for visualization
CLASS_COLORS = {
    "cross": (0, 255, 0), 
    "egg": (255, 0, 0), 
    "orange ball": (0, 140, 255), 
    "white ball": (255, 255, 255), 
    "robothead": (0, 0, 255), 
    "robottail": (255, 0, 255)
}

# Vision system settings
ORIENTED_OBJECTS = ["robothead", "robottail", "egg", "cross"]
EGG_OBB_ASPECT_RATIO_THRESHOLD = 1.3

# Navigation parameters - no sensors, only vision navigation
COMMAND_COOLDOWN = 0.35  # CAREFUL: Increased from 0.3 to 0.5 for better measurement between commands
PRINT_INTERVAL = 3      # Faster status updates
TURN_THRESHOLD = 10     # minimum number of degrees to turn.
DISTANCE_THRESHOLD = 30 # cm - stop 30 cm from ball for precise correction
MAX_FORWARD_DISTANCE = 30 # maximum length for forward movement

# Ball pickup sequence - hardcoded distances
PICKUP_FORWARD_DISTANCE = 35  # cm - distance to drive forward to collect ball
PICKUP_BACKWARD_DISTANCE = 35  # cm - distance to back up after collection

ROBOT_TURN_SPEED = 20 # FASTER: Motor speed for turning (increased from 10 to 25)
ROBOT_FORWARD_SPEED = 35 # FASTER: Motor speed for forward movement (increased from 30 to 50)  
ESTIMATED_TURN_RATE = 180.0  # degrees per second - updated based on wheel calibration (half rotation = 90°)