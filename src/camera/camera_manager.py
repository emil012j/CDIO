# -*- coding: utf-8 -*-
"""
 Initialiserer kamera, læser frames, tegner detection bokse, viser status - OPTIMERET VERSION
"""

import cv2
import time
import math
import threading
import numpy as np
from ..config.settings import *
from .track_calibration import TrackCalibrator

#kamera manager, der håndterer kameraet og viser det på skærmen - MED PERFORMANCE OPTIMERING
class CameraManager:
    #initialiserer kameraet
    def __init__(self, camera_source=CAMERA_SOURCE):
        self.camera_source = camera_source
        self.cap = None
        self.is_initialized = False
        self.last_frame_time = 0
        self.target_frame_interval = 1.0 / TARGET_FPS # FPS begrænsning
        self.track_calibrator = TrackCalibrator()
        
    #initialiserer kameraet med de rigtige indstillinger - OPTIMERET
    def initialize_camera(self):
        """Initialize the camera with optimized settings"""
        print("Initializing camera with optimized settings...")
        try:
            # Try with CAP_DSHOW first (Windows DirectShow)
            self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                print("Trying backup camera...")
                self.cap = cv2.VideoCapture(self.camera_source)
            if not self.cap.isOpened():
                print("ERROR: Could not open camera {}".format(self.camera_source))
                return False
            
            # Set camera resolution and settings
            width, height = CAMERA_RESOLUTION
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # Performance optimizations
            self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
            
            # Test frame capture
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("ERROR: Could not read frame from camera")
                return False
            
            print("Camera initialized successfully - Resolution: {}x{}, Target FPS: {}".format(
                int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                TARGET_FPS
            ))
            
            self.is_initialized = True
            return True
            
        except Exception as e:
            print("ERROR initializing camera: {}".format(e))
            return False
    
    #læser frames fra kameraet - MED FPS BEGRÆNSNING
    def read_frame(self):
        """Read a frame from the camera with FPS limiting"""
        if not self.is_initialized or self.cap is None:
            return None
            
        # FPS throttling
        current_time = time.time()
        time_since_last = current_time - self.last_frame_time
        if time_since_last < self.target_frame_interval:
            time.sleep(max(0, self.target_frame_interval - time_since_last))
        
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                self.last_frame_time = time.time()
                return frame
            return None
        except Exception:
            return None
    
    #afslutter kameraet ved at trykke escape
    def release(self):
        """Release the camera"""
        if self.cap is not None:
            self.cap.release()
            self.is_initialized = False
            print("Camera released")
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.release()

    def calibrate_track(self) -> bool:
        """Start track calibration process"""
        if self.cap is None:
            print("Camera not initialized for calibration")
            return False
        return self.track_calibrator.start_calibration(self.cap)

    #tegner detection bokse
    def draw_detection_box(self, frame, position, label, color):
        """Draw a detection box with label"""
        try:
            cv2.circle(frame, position, 10, color, -1)
            cv2.putText(frame, label, (position[0]+10, position[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        except Exception:
            pass

    #tegner navigation information som feks. vinklen mellem robot og bold, og afstanden til bold
    def draw_navigation_info(self, frame, robot_center, target_ball, robot_heading, target_heading):
        """Draw navigation information on the frame"""
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
                    end_x = int(robot_center[0] + arrow_length * np.cos(np.radians(robot_heading)))
                    end_y = int(robot_center[1] + arrow_length * np.sin(np.radians(robot_heading)))
                    cv2.arrowedLine(frame, robot_center, (end_x, end_y), (0, 0, 255), 3)
                
                if target_heading is not None:
                    # Target heading arrow (green)  
                    arrow_length = 70
                    end_x = int(robot_center[0] + arrow_length * np.cos(np.radians(target_heading)))
                    end_y = int(robot_center[1] + arrow_length * np.sin(np.radians(target_heading)))
                    cv2.arrowedLine(frame, robot_center, (end_x, end_y), (0, 255, 0), 2)
                    
        except Exception:
            pass

    #viser status information på skærmen som feks. om robotten har head, tail og bold, og skaleringsfaktor
    def display_status(self, frame, robot_head, robot_tail, balls, scale_factor, navigation_info=None):
        """Display status information on the frame"""
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