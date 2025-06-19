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
from ..camera.goal_calibrator import GoalCalibrator


#kamera manager, der håndterer kameraet og viser det på skærmen
class CameraManager:
    #initialiserer kameraet
    def __init__(self):
        self.cap = None
        self.is_initialized = False
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.mapx = None
        self.mapy = None
        self.goal_calibrator = GoalCalibrator()
        

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
            if ret and frame is not None:
                # Apply undistortion if calibration data is available
                return self.undistort_frame(frame)
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

    def undistort_frame(self, frame): # Denne funktion var den som printede det lort med "WARNING: Camera calibration data not found"
        """Undistort frame using calibration data"""
        if self.camera_matrix is None or self.distortion_coefficients is None:
            
            return frame
            
        try:
            # Get frame dimensions
            h, w = frame.shape[:2]
            
            # Calculate new camera matrix with balance between undistortion and keeping pixels
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, 
                self.distortion_coefficients,
                (w, h), 
                alpha=0.925  # Slight adjustment to help preserve proportions
            )
            
            # Apply undistortion
            undistorted = cv2.undistort(
                frame,
                self.camera_matrix,
                self.distortion_coefficients,
                None,
                newcameramtx
            )
            
            # Only crop if we have a very good ROI
            x, y, w, h = roi
            if all(v > 0 for v in [x, y, w, h]) and w > frame.shape[1]*0.9 and h > frame.shape[0]*0.9:
                undistorted = undistorted[y:y+h, x:x+w]
                undistorted = cv2.resize(undistorted, (frame.shape[1], frame.shape[0]))
            
            return undistorted
            
        except Exception as e:
            print(f"ERROR during undistortion: {e}")
            return frame

    #tegner detection bokse
def draw_detection_box(frame, position, label, color):
    try:
        cv2.circle(frame, position, 10, color, -1)  # Tegner detection bokse
        cv2.putText(frame, label, (position[0] + 15, position[1] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    except Exception:
        pass

def calibrate_goal(self):
        """Start goal calibration process"""
        if self.cap is None:
            print("Camera not initialized for calibration")
            return False
        # Note: start_calibration doesn't return a value, it handles its own UI loop
        self.goal_calibrator.start_calibration(self.cap)

#tegner navigation information som feks. vinklen mellem robot og bold, og afstanden til bold
def draw_navigation_info(frame, robot_center, target_ball, robot_heading, target_heading, navigation_info=None):
    try:
        if robot_center and target_ball:
            # Draw robot center
            cv2.circle(frame, robot_center, 10, (255, 0, 255), -1)
            
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
            
    except Exception:
        pass 