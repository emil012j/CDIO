# -*- coding: utf-8 -*-
"""
Indstillinger for robotten, feks. IP, port, kamera, modeller, farver, etc.
"""

# Netværk indstillinger (IP, port)
ROBOT_IP = "192.168.65.158"
COMMAND_PORT = 1233

# Kamera indstillinger (resolution, confidence)
CAMERA_SOURCE = 1
CAMERA_RESOLUTION = (1280, 720)
CONFIDENCE_THRESHOLD = 0.35 #0.35

# Memory management indstillinger (forhindrer lag ved mange objekter)
MAX_OBJECTS_BEFORE_SKIP = 4      # Reduceret fra 6 - tidligere intervention
MEMORY_GC_INTERVAL = 5.0         # Reduceret fra 10 - mere hyppig cleanup  
FRAME_SKIP_RATIO = 2             # Skip hver 2. frame ved mange objekter
EMERGENCY_OBJECT_LIMIT = 8       # Hvis flere objekter: KØR UDEN VISUALIZATION

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
WHEEL_DIAMETER_CM = 68.8  # Korrekt hjuldiameter som oplyst
WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159  # ~216.06 cm per rotation

# Hastigheds indstillinger - forskellige hastigheder til forskellige opgaver
ROBOT_TURN_SPEED = 40      # Standard drejningshastighed
ROBOT_TURN_SPEED_SLOW = 20 # Langsom drejning til præcis sigtning
ROBOT_FORWARD_SPEED = 50   # Standard kørehastighed  
ROBOT_FORWARD_SPEED_SLOW = 25 # Langsom kørsel når tæt på mål

# Harvester motor indstillinger
POWER_HARVESTER = 45  # Speed 45 konstant (positiv for baglæns på denne motor)

# Kalibrerede bevægelsesrater baseret på korrekt hjuldiameter
# En halv omgang af hjulene = 90 graders drejning (som oplyst)
# 0.25 omgang = 45 grader, 0.125 omgang = 22.5 grader osv.
MOTOR_ROTATIONS_PER_90_DEGREES = 0.5  # Halv omgang = 90° rotation
ESTIMATED_FORWARD_RATE = WHEEL_CIRCUMFERENCE_CM / 10.8  # Baseret på faktisk hjulomkreds

# Præcisionsnavigation parametre - mindre forsigtige værdier
COARSE_TURN_THRESHOLD = 15   # Grov justering: store drejninger  
FINE_TURN_THRESHOLD = 8      # Fin justering: små korrektioner  
VERY_FINE_TURN_THRESHOLD = 4 # Meget fin justering når tæt på
DISTANCE_THRESHOLD = 5       # Stopafstand til bold - øget fra 3
PRECISION_DISTANCE = 20      # Afstand hvor præcisionskørsel starter - øget

# Progressive korrektur indstillinger - mindre forsigtige
MAX_TURN_COARSE = 60         # Store drejninger til grovkorrektion - øget
MAX_TURN_FINE = 25           # Mindre drejninger til finjustering - øget  
MAX_TURN_PRECISION = 10      # Meget små drejninger når tæt på - øget

# Fremdrift parametre - længere distancer for mindre forsigtig kørsel
MAX_FORWARD_DISTANCE_FAR = 50   # Lange køretur når langt væk - øget betydeligt
MAX_FORWARD_DISTANCE_NEAR = 20  # Korte køretur når tæt på - øget
MAX_FORWARD_DISTANCE_CLOSE = 10 # Meget korte køretur når meget tæt på - øget

# PID Settings (vi bruger det ikke gyro indtil videre - kun hvis vi vil bruge gyro til præcis navigation)
PID_KP = 0.5   # Proportional - hvor hurtigt robotten reagerer på fejl (højere = mere aggressiv)
PID_KI = 0.01  # Integral - husker gamle fejl og korrigerer langsomt (forhindrer permanent fejl)
PID_KD = 0.3   # Derivative - forudser fremtidige fejl (forhindrer oversving)