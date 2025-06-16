# -*- coding: utf-8 -*-
"""
 Initialiserer kamera, læser frames, tegner detection bokse, viser status
"""

import cv2
import time
import math
import pickle
import numpy as np
from ..config.settings import *

#kamera manager, der håndterer kameraet og viser det på skærmen
class CameraManager:
    #initialiserer kameraet
    def __init__(self):
        self.cap = None
        self.is_initialized = False
        
    def initialize_camera(self):
        print("Initializing camera...")
        try:
            # Try to open camera with a timeout
            self.cap = cv2.VideoCapture(CAMERA_SOURCE, cv2.CAP_DSHOW)  # Add DSHOW backend explicitly
            
            # Wait a short time for camera to initialize
            time.sleep(0.5)
            
            if self.cap is None:
                print("ERROR: Camera object is None")
                return False
                
            if not self.cap.isOpened():
                print("ERROR: Camera failed to open")
                return False
            
            # Set camera resolution og indstillinger
            width, height = CAMERA_RESOLUTION
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # Test frame capture with timeout
            start_time = time.time()
            while time.time() - start_time < 2.0:  # 2 second timeout
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print("Camera initialized successfully - Resolution: {}x{}".format(
                        actual_width, actual_height
                    ))
                    self.is_initialized = True
                    return True
                time.sleep(0.1)
            
            print("ERROR: Could not read frame from camera within timeout")
            return False
            
        except Exception as e:
            print("ERROR initializing camera:", str(e))
            if self.cap is not None:
                self.cap.release()
            return False
    
    #læser frames fra kameraet
    def read_frame(self):
        if not self.is_initialized or self.cap is None:
            return None
            
        try:
            ret, frame = self.cap.read()  # Læser frames fra kamera
            return frame if ret and frame is not None else None
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
def draw_navigation_info(frame, robot_center, target_ball, robot_heading, target_heading, navigation_info=None):
    try:
        if robot_center and target_ball:
            # Draw line from robot to original target (dotted yellow)
            cv2.line(frame, robot_center, target_ball, (0, 255, 255), 1, cv2.LINE_AA)
            
            # If we have corrected target from height adjustment, draw that too
            if navigation_info and 'corrected_target' in navigation_info:
                corrected = navigation_info['corrected_target']
                # Draw line to corrected position (solid green)
                cv2.line(frame, robot_center, corrected, (0, 255, 0), 2)
                # Draw small circle at corrected position
                cv2.circle(frame, corrected, 5, (0, 255, 0), -1)
                # Draw small circle at original position
                cv2.circle(frame, target_ball, 5, (0, 255, 255), -1)
            
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
        # Show status information
        y_offset = 30
        cv2.putText(frame, "Robot head: {}".format("YES" if robot_head else "NO"), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (0, 255, 0) if robot_head else (0, 0, 255), 2)
        
        y_offset += 30
        cv2.putText(frame, "Robot tail: {}".format("YES" if robot_tail else "NO"), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (0, 255, 0) if robot_tail else (0, 0, 255), 2)
        
        y_offset += 30
        cv2.putText(frame, "Balls found: {}".format(len(balls)), 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (0, 255, 0) if balls else (0, 0, 255), 2)
        
        y_offset += 30
        scale_text = "Scale: {:.2f} mm/px".format(scale_factor) if scale_factor else "Scale: NO"
        cv2.putText(frame, scale_text, 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                   (0, 255, 0) if scale_factor else (0, 0, 255), 2)
        
        # Navigation info
        if navigation_info:
            y_offset += 40
            cv2.putText(frame, "Robot heading: {:.1f}deg".format(navigation_info["robot_heading"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25  
            cv2.putText(frame, "Target heading: {:.1f}deg".format(navigation_info["target_heading"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25
            cv2.putText(frame, "Angle diff: {:.1f}deg".format(navigation_info["angle_diff"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            y_offset += 25  
            cv2.putText(frame, "Distance: {:.1f} cm".format(navigation_info["distance_cm"]), 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Add height correction info
            if 'corrected_target' in navigation_info:
                y_offset += 25
                cv2.putText(frame, "Height Correction: ACTIVE", 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                           
    except Exception:
        pass 