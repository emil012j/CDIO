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
    def __init__(self, camera_source=CAMERA_SOURCE):
        self.camera_source = camera_source
        self.cap = None
        self.is_initialized = False
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.load_calibration_data()
        
    def load_calibration_data(self):
        """Load camera calibration data from text files"""
        try:
            # Load camera matrix and reshape to 3x3
            self.camera_matrix = np.loadtxt('src/camera/camera_matrix.txt').reshape(3, 3)
            # Load distortion coefficients and reshape to 5x1
            self.distortion_coefficients = np.loadtxt('src/camera/distortion_coefficients.txt').reshape(5, 1)
            
            print("Successfully loaded calibration data:")
            print("Camera Matrix shape:", self.camera_matrix.shape)
            print("Distortion Coefficients shape:", self.distortion_coefficients.shape)
            print("Camera Matrix:\n", self.camera_matrix)
            print("Distortion Coefficients:\n", self.distortion_coefficients)
            
        except Exception as e:
            print(f"Could not load calibration data: {e}")
            self.camera_matrix = None
            self.distortion_coefficients = None

    def save_calibration_data(self, camera_matrix, distortion_coefficients):
        """Save camera calibration data to pickle file"""
        try:
            calibration_data = {
                'camera_matrix': camera_matrix,
                'distortion_coefficients': distortion_coefficients
            }
            with open('src/camera/calibration_data.pkl', 'wb') as f:
                pickle.dump(calibration_data, f)
            print("Successfully saved calibration data")
            return True
        except Exception as e:
            print(f"Could not save calibration data: {e}")
            return False

    #initialiserer kameraet med de rigtige indstillinger
    def initialize_camera(self):
        print("Initializing camera...")
        try:
            # Prøv med CAP_DSHOW først (Windows DirectShow)
            self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_DSHOW)  # Initialiserer kamera
            if not self.cap.isOpened():
                print("Proever backup kamera...")
                self.cap = cv2.VideoCapture(self.camera_source)
            if not self.cap.isOpened():
                print("ERROR: Could not open camera {}".format(self.camera_source))
                return False
            
            # Set camera resolution og indstillinger
            width, height = CAMERA_RESOLUTION
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
            
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

    def calibrate_camera(self, num_images=10, pattern_size=(9,6), square_size=20.0):
        """
        Calibrate camera using chessboard pattern
        num_images: Number of calibration images to capture
        pattern_size: Size of chessboard pattern (width, height)
        square_size: Size of each square in mm
        """
        if not self.is_initialized:
            print("Camera not initialized")
            return False

        # Prepare object points
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)
        objp = objp * square_size

        # Arrays to store object points and image points
        objpoints = [] # 3D points in real world space
        imgpoints = [] # 2D points in image plane

        print("Starting camera calibration...")
        print("Press 'c' to capture calibration image, 'q' to quit")

        images_captured = 0
        while images_captured < num_images:
            frame = self.read_frame()
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            # Draw and display the corners
            if ret:
                cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
                cv2.putText(frame, f"Found pattern! Press 'c' to capture ({images_captured}/{num_images})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No pattern found", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow('Calibration', frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('c') and ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                images_captured += 1
                print(f"Captured image {images_captured}/{num_images}")
            elif key == ord('q'):
                break

        cv2.destroyAllWindows()

        if len(objpoints) < 3:
            print("Not enough calibration images captured")
            return False

        print("Calculating camera matrix and distortion coefficients...")
        # Convert to numpy arrays and ensure correct types
        objpoints = [np.array(points, dtype=np.float32) for points in objpoints]
        imgpoints = [np.array(points, dtype=np.float32) for points in imgpoints]
        image_size = tuple(gray.shape[::-1])
        
        # Initialize camera matrix and distortion coefficients
        camera_matrix = np.zeros((3, 3), dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, camera_matrix, dist_coeffs
        )

        if ret:
            print("Calibration successful!")
            # Save calibration data
            if self.save_calibration_data(mtx, dist):
                self.camera_matrix = mtx
                self.distortion_coefficients = dist
                return True
        else:
            print("Calibration failed")
            return False

    def undistort_frame(self, frame):
        """Undistort frame using calibration data"""
        if self.camera_matrix is None or self.distortion_coefficients is None:
            return frame
            
        try:
            # Get optimal camera matrix
            h, w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.distortion_coefficients, (w,h), 1, (w,h)
            )
            
            # Undistort
            dst = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients, None, newcameramtx)
            
            # Crop the image
            x, y, w, h = roi
            if all(v > 0 for v in [x, y, w, h]):
                dst = dst[y:y+h, x:x+w]
                
            return dst
        except Exception as e:
            print(f"Error during undistortion: {e}")
            return frame

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