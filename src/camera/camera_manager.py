# -*- coding: utf-8 -*-
"""
 Initialiserer kamera, læser frames, tegner detection bokse, viser status
"""

import cv2
import time
import math
from ..config.settings import *

#kamera manager, der håndterer kameraet og viser det på skærmen
class CameraManager:
    #initialiserer kameraet
    def __init__(self, camera_source=CAMERA_SOURCE):
        self.camera_source = camera_source
        self.cap = None
        self.is_initialized = False
        
    #initialiserer kameraet med de rigtige indstillinger
    def initialize_camera(self):
        print("Initializing camera...")
        try:
            self.cap = cv2.VideoCapture(self.camera_source)  # Initialiserer kamera
            if not self.cap.isOpened():
                print("ERROR: Could not open camera {}".format(self.camera_source))
                return False
            
            # Set camera resolution
            width, height = CAMERA_RESOLUTION
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # Test frame capture
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("ERROR: Could not read frame from camera")
                return False
            
            print("Camera initialized successfully - Resolution: {}x{}".format(
                int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            ))
            
            self.is_initialized = True
            return True
            
        except Exception as e:
            print("ERROR initializing camera: {}".format(e))
            return False
    
    #læser frames fra kameraet
    def read_frame(self):
        if not self.is_initialized or self.cap is None:
            return None
            
        try:
            ret, frame = self.cap.read()  # Læser frames fra kamera
            if ret and frame is not None:
                return frame
            return None
        except Exception:
            return None
    
    #afslutter kameraet ved at trykke escape
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.is_initialized = False
            print("Camera released")
    
    def __del__(self):
        """Destructor to ensure camera is released"""
        self.release()

    #tegner detection bokse
def draw_detection_box(frame, position, label, color):
    try:
        cv2.circle(frame, position, 10, color, -1)  # Tegner detection bokse
        cv2.putText(frame, label, (position[0] + 15, position[1] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    except Exception:
        pass

#tegner navigation information som feks. vinklen mellem robot og bold, og afstanden til bold
def draw_navigation_info(frame, robot_center, target_ball, robot_heading, target_heading):
    try:
        if robot_center and target_ball:
            # Draw line from robot to target
            cv2.line(frame, robot_center, target_ball, (0, 255, 255), 3)
            
            # Draw robot center
            cv2.circle(frame, robot_center, 10, (255, 0, 255), -1)
            
            # Draw heading arrows
            if robot_heading is not None:
                # Current heading arrow (red)
                arrow_length = 50
                end_x = int(robot_center[0] + arrow_length * math.cos(math.radians(robot_heading)))
                end_y = int(robot_center[1] + arrow_length * math.sin(math.radians(robot_heading)))
                cv2.arrowedLine(frame, robot_center, (end_x, end_y), (0, 0, 255), 3)
            
            if target_heading is not None:
                # Target heading arrow (green)  
                arrow_length = 70
                end_x = int(robot_center[0] + arrow_length * math.cos(math.radians(target_heading)))
                end_y = int(robot_center[1] + arrow_length * math.sin(math.radians(target_heading)))
                cv2.arrowedLine(frame, robot_center, (end_x, end_y), (0, 255, 0), 2)
                
    except Exception:
        pass

#viser status information på skærmen som feks. om robotten har head, tail og bold, og skaleringsfaktor
def display_status(frame, robot_head, robot_tail, balls, scale_factor, navigation_info=None):
    try:
        # Viser status information på skærm
        y_offset = 30
        cv2.putText(frame, "Robot head: {}".format("YES" if robot_head else "NO"), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if robot_head else (0, 0, 255), 2)
        
        y_offset += 30
        cv2.putText(frame, "Robot tail: {}".format("YES" if robot_tail else "NO"), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if robot_tail else (0, 0, 255), 2)
        
        y_offset += 30
        cv2.putText(frame, "Balls found: {}".format(len(balls)), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if balls else (0, 0, 255), 2)
        
        y_offset += 30
        scale_text = "Scale: {:.2f} mm/px".format(scale_factor) if scale_factor else "Scale: NO"
        cv2.putText(frame, scale_text, 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if scale_factor else (0, 0, 255), 2)
        
        # Navigation info
        if navigation_info:
            y_offset += 40
            cv2.putText(frame, "Robot heading: {:.1f}°".format(navigation_info["robot_heading"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25  
            cv2.putText(frame, "Target heading: {:.1f}°".format(navigation_info["target_heading"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25
            cv2.putText(frame, "Angle diff: {:.1f}°".format(navigation_info["angle_diff"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25  
            cv2.putText(frame, "Distance: {:.1f} cm".format(navigation_info["distance_cm"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                       
    except Exception:
        pass 