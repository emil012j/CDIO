import cv2
import numpy as np
import json
import os
from typing import List, Tuple, Optional, Dict

class TrackCalibrator:
    def __init__(self, window_name: str = "Track Calibration"):
        self.window_name = window_name
        self.boundary_points: List[Tuple[int, int]] = []
        self.calibration_mode = False
        self.boundary_file = 'track_boundaries.json'
        
    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        """Handle mouse events for calibration"""
        if event == cv2.EVENT_LBUTTONDOWN and self.calibration_mode:
            if len(self.boundary_points) < 4:
                self.boundary_points.append((x, y))
                print("Added point {}: ({}, {})".format(len(self.boundary_points), x, y))
            if len(self.boundary_points) == 4:
                self.save_boundaries()
                print("Calibration complete! Press 'q' to continue.")

    def add_boundary_point(self, x: int, y: int):
        """Add a boundary point"""
        self.boundary_points.append((x, y))
        print("Added point {}: ({}, {})".format(len(self.boundary_points), x, y))
    
    def save_boundaries(self):
        """Save boundary points to file"""
        try:
            with open(self.boundary_file, 'w') as f:
                json.dump({
                    'boundary_points': self.boundary_points,
                    'calibrated': True
                }, f)
            print("Boundary points saved to {}".format(self.boundary_file))
        except Exception as e:
            print("Error saving boundary points: {}".format(e))
    
    def load_boundaries(self) -> bool:
        """Load boundary points from file"""
        try:
            if os.path.exists(self.boundary_file):
                with open(self.boundary_file, 'r') as f:
                    data = json.load(f)
                    self.boundary_points = data.get('boundary_points', [])
                    return data.get('calibrated', False)
        except Exception as e:
            print("Error loading boundary points: {}".format(e))
        return False

    def draw_calibration_overlay(self, frame: np.ndarray) -> None:
        """Draw calibration points and instructions on the frame"""
        # Draw current points
        for i, point in enumerate(self.boundary_points):
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
            cv2.putText(frame, str(i+1), (point[0]+10, point[1]+10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw lines between points
        if len(self.boundary_points) > 1:
            for i in range(len(self.boundary_points)-1):
                cv2.line(frame, self.boundary_points[i], 
                        self.boundary_points[i+1], (0, 255, 0), 2)
        
        # Draw line from last point to first if we have 3 points
        if len(self.boundary_points) == 3:
            cv2.line(frame, self.boundary_points[2], 
                    self.boundary_points[0], (0, 255, 0), 2)
        
        # Show instructions
        cv2.putText(frame, "Click to set points (1-4)", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press 'r' to reset", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press 'q' when done", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    def start_calibration(self, cap: cv2.VideoCapture) -> bool:
        """Start the calibration process"""
        if not cap.isOpened():
            print("Camera not initialized")
            return False

        self.calibration_mode = True
        self.boundary_points = []
        
        # Create window and set mouse callback
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        print("\nCalibration Mode:")
        print("1. Click to set the four corners of your track in this order:")
        print("   - Top Left")
        print("   - Top Right")
        print("   - Bottom Right")
        print("   - Bottom Left")
        print("2. Press 'q' when done to save and continue")
        print("3. Press 'r' to reset if you make a mistake\n")
        
        while self.calibration_mode:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Draw calibration overlay
            self.draw_calibration_overlay(frame)
            
            cv2.imshow(self.window_name, frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                if len(self.boundary_points) == 4:
                    self.save_boundaries()
                    self.calibration_mode = False
                else:
                    print("Please set all 4 points before quitting")
            elif key == ord('r'):
                self.boundary_points = []
                print("Reset calibration points")
        
        cv2.destroyWindow(self.window_name)
        return len(self.boundary_points) == 4

def main():
    """Simple standalone calibration tool"""
    print("Starting track calibration tool...")
    
    # Initialize camera
    cap = cv2.VideoCapture(0)  # Use default camera
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
        
    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # Create calibrator and start calibration
    calibrator = TrackCalibrator()
    if calibrator.start_calibration(cap):
        print("Calibration completed successfully!")
    else:
        print("Calibration was cancelled or failed")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 