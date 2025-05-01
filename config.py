"""
Configuration file for the EV3 robot navigation system

Contains default settings that can be adjusted for different environments
"""

# Camera settings
CAMERA_CONFIG = {
    "model_path": "src/camera/best.pt",   # Path to the YOLO model
    "camera_id": 1,                        # Camera device ID
    "resolution": (1280, 720),             # Camera resolution
    "confidence": 0.35,                    # Minimum confidence for detections
    "host": "0.0.0.0",                     # Server listen address (all interfaces)
    "port": 5000                           # Server port
}

# Robot settings
ROBOT_CONFIG = {
    "server_ip": "192.168.8.97",          # Default server IP to connect to
    "server_port": 5000,                   # Default server port
    "reconnect_attempts": 5                # Number of reconnection attempts
}

# Motor settings
MOTOR_CONFIG = {
    "left_motor_port": "OUTPUT_A",         # Left motor port
    "right_motor_port": "OUTPUT_D",        # Right motor port
    "pickup_motor_port": "OUTPUT_C",       # Pickup mechanism motor port
    "default_speed": 50,                   # Default motor speed (0-100)
    "turning_speed": 30                    # Default turning speed (0-100)
}

# Sensor settings
SENSOR_CONFIG = {
    "touch_sensor_port": "INPUT_1",        # Touch sensor port
    "color_sensor_port": "INPUT_4"         # Color sensor port
}

# Navigation settings
NAVIGATION_CONFIG = {
    "target_precision": 20,                # Distance threshold to consider target reached
    "min_distance": 50,                    # Minimum distance to consider for movement
    "avoid_distance": 100,                 # Distance to maintain from obstacles
    "collect_white_balls": True,           # Whether to collect white balls
    "collect_orange_balls": True,          # Whether to collect orange balls
    "avoid_eggs": True                     # Whether to avoid eggs
}

# Timing settings
TIMING_CONFIG = {
    "update_interval": 0.05,               # Main loop update interval (seconds)
    "data_timeout": 3.0                    # How long to wait before stopping with no data
}

# Debug settings
DEBUG_CONFIG = {
    "show_camera_feed": True,              # Whether to show camera feed window
    "show_detections": True,               # Whether to show detection rectangles
    "show_coordinates": True,              # Whether to show coordinate overlay
    "verbose_logging": False               # Whether to log verbose information
}

def load_config():
    """
    Load configuration from config.py
    Could be extended to load from a file
    """
    return {
        "camera": CAMERA_CONFIG,
        "robot": ROBOT_CONFIG,
        "motor": MOTOR_CONFIG,
        "sensor": SENSOR_CONFIG,
        "navigation": NAVIGATION_CONFIG,
        "timing": TIMING_CONFIG,
        "debug": DEBUG_CONFIG
    } 