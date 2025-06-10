"""
Goal calibration - click 2 points to mark the goal edges (top and bottom of goal opening)
"""

import cv2
import json
import os
from ..config.settings import *

class GoalCalibrator:
    def __init__(self):
        self.goal_file = "goal_positions.json"
        self.goal_points = []  # Will store 2 points: [top_goal_edge, bottom_goal_edge]
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to set goal edge points"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.goal_points) < 2:
                self.goal_points.append((x, y))
                print("Goal edge point {} set at: ({}, {})".format(len(self.goal_points), x, y))
                
                if len(self.goal_points) == 2:
                    print("Both goal edges captured! Press 'q' to save and exit")
            else:
                print("Both goal edges already set. Press 'r' to reset")

    def save_goal_positions(self) -> bool:
        """Save goal edge positions to file"""
        if len(self.goal_points) != 2:
            print("Need exactly 2 goal edge points")
            return False
            
        try:
            # Calculate goal center and target position
            top_edge = self.goal_points[0]
            bottom_edge = self.goal_points[1]
            
            # Goal center (middle between the two edges)
            goal_center_x = (top_edge[0] + bottom_edge[0]) // 2
            goal_center_y = (top_edge[1] + bottom_edge[1]) // 2
            
            # Target position (30cm away from goal center)
            target_x = goal_center_x + 80  # Move 80 pixels away from goal (toward field)
            target_y = goal_center_y
            
            goal_data = {
                'calibrated': True,
                'top_edge': top_edge,
                'bottom_edge': bottom_edge,
                'goal_center': (goal_center_x, goal_center_y),
                'target_position': (target_x, target_y)
            }
            
            with open(self.goal_file, 'w') as f:
                json.dump(goal_data, f, indent=2)
                
            print("Goal positions saved to {}".format(self.goal_file))
            print("Goal center: ({}, {})".format(goal_center_x, goal_center_y))
            print("Target position: ({}, {})".format(target_x, target_y))
            return True
            
        except Exception as e:
            print("Error saving goal positions: {}".format(e))
            return False

    def load_goal_positions(self) -> bool:
        """Load goal positions from file"""
        try:
            if os.path.exists(self.goal_file):
                with open(self.goal_file, 'r') as f:
                    data = json.load(f)
                    if data.get('calibrated', False):
                        self.goal_points = [data['top_edge'], data['bottom_edge']]
                        return True
        except Exception as e:
            print("Error loading goal positions: {}".format(e))
        return False

    def draw_calibration_overlay(self, frame):
        """Draw goal calibration points and instructions"""
        # Draw current goal edge points
        for i, point in enumerate(self.goal_points):
            cv2.circle(frame, point, 8, (0, 255, 0), -1)
            label = "Top Edge" if i == 0 else "Bottom Edge"
            cv2.putText(frame, label, (point[0]+15, point[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw line between points if we have both
        if len(self.goal_points) == 2:
            cv2.line(frame, self.goal_points[0], self.goal_points[1], (0, 255, 0), 3)
            
            # Calculate and show goal center
            center_x = (self.goal_points[0][0] + self.goal_points[1][0]) // 2
            center_y = (self.goal_points[0][1] + self.goal_points[1][1]) // 2
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Goal Center", (center_x+10, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Show target position
            target_x = center_x + 80
            target_y = center_y
            cv2.circle(frame, (target_x, target_y), 8, (0, 0, 255), -1)
            cv2.putText(frame, "Target Position", (target_x+10, target_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.line(frame, (center_x, center_y), (target_x, target_y), (0, 0, 255), 2)
        
        # Show instructions
        instructions = [
            "Click TOP edge of goal",
            "Click BOTTOM edge of goal", 
            "Press 'r' to reset",
            "Press 'q' to save and exit"
        ]
        
        for i, instruction in enumerate(instructions):
            color = (0, 255, 0) if i < len(self.goal_points) else (255, 255, 255)
            cv2.putText(frame, instruction, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def start_calibration(self, cap):
        """Start goal calibration process"""
        print("\n=== GOAL CALIBRATION ===")
        print("Click the TOP edge of the goal")
        print("Then click the BOTTOM edge of the goal")
        print("Press 'r' to reset, 'q' to save and exit")
        
        # Load existing calibration if available
        if self.load_goal_positions():
            print("Loaded existing goal calibration")
        
        cv2.namedWindow("Goal Calibration")
        cv2.setMouseCallback("Goal Calibration", self.mouse_callback)
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                self.draw_calibration_overlay(frame)
                cv2.imshow("Goal Calibration", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    if len(self.goal_points) == 2:
                        if self.save_goal_positions():
                            break
                    else:
                        print("Need 2 goal edge points before saving")
                elif key == ord('r'):
                    self.goal_points = []
                    print("Goal points reset")
                elif key == 27:  # ESC
                    break
                    
        finally:
            cv2.destroyWindow("Goal Calibration")
            
        return len(self.goal_points) == 2

# For backward compatibility, keep the old class name
TrackCalibrator = GoalCalibrator

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