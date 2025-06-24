"""
Simple goal position click-calibration
"""

import cv2
import json
import os

class GoalCalibrator:
    def __init__(self):
        self.goal_file = "goal_positions.json"
        self.goal_position = None
        self.goal_waypoint = None
        self.calibrating_mode = 'goal' # 'goal' or 'waypoint'
        self._load_existing_data() # Load existing data at init

    def _load_existing_data(self):
        """Load existing goal positions and waypoint without overwriting"""
        try:
            if os.path.exists(self.goal_file):
                with open(self.goal_file, 'r') as f:
                    data = json.load(f)
                    if 'goal_position' in data:
                        self.goal_position = tuple(data['goal_position'])
                    if 'goal_waypoint' in data:
                        self.goal_waypoint = tuple(data['goal_waypoint'])
        except Exception as e:
            print(f"Error loading existing goal data: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """Click to set goal position or waypoint"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.calibrating_mode == 'goal':
                self.goal_position = (x, y)
                print(f"Goal position set to: ({x}, {y}) - press 'q' to save")
            elif self.calibrating_mode == 'waypoint':
                self.goal_waypoint = (x, y)
                print(f"Waypoint set to: ({x}, {y}) - press 'q' to save")

    def save_goal(self):
        """Save goal position and waypoint"""
        data = {'calibrated': True}
        if self.goal_position:
            data['goal_position'] = [self.goal_position[0], self.goal_position[1]]
        if self.goal_waypoint:
            data['goal_waypoint'] = [self.goal_waypoint[0], self.goal_waypoint[1]]

        if not self.goal_position and not self.goal_waypoint:
            print("No goal or waypoint set to save.")
            return False

        try:
            with open(self.goal_file, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"Calibration data saved to {self.goal_file}")
            return True
        except Exception as e:
            print(f"Failed to save calibration data: {e}")
            return False

    def start_calibration(self, frame):
        """Start click calibration with proper event loop"""
        print("Click to set position. Press 'g' for Goal, 'w' for Waypoint. 'q' to save, 'ESC' to cancel")
        cv2.namedWindow("Goal Calibration")
        cv2.setMouseCallback("Goal Calibration", self.mouse_callback)
        
        # Calibration loop
        while True:
            display_frame = frame.copy()
            
            # Draw current goal position if set
            if self.goal_position:
                cv2.circle(display_frame, self.goal_position, 15, (0, 255, 0), -1)
                cv2.putText(display_frame, f"GOAL: {self.goal_position}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw current waypoint if set
            if self.goal_waypoint:
                cv2.circle(display_frame, self.goal_waypoint, 15, (0, 255, 255), 2)
                cv2.putText(display_frame, f"WAYPOINT: {self.goal_waypoint}", 
                           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Display current mode
            cv2.putText(display_frame, f"Mode: {self.calibrating_mode.upper()}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "Click to set position. Press 'g' for Goal, 'w' for Waypoint.", (10, 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, "Press 'q' to save, ESC to cancel", (10, 160), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("Goal Calibration", display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Save data
                if self.save_goal():
                    print("Calibration data successfully saved!")
                    break
                else:
                    print("Failed to save calibration data!")
            elif key == ord('g'): # Switch to goal calibration
                self.calibrating_mode = 'goal'
                print("Switched to GOAL calibration mode")
            elif key == ord('w'): # Switch to waypoint calibration
                self.calibrating_mode = 'waypoint'
                print("Switched to WAYPOINT calibration mode")
            elif key == 27:  # ESC key - cancel
                print("Goal calibration cancelled")
                break
        
        cv2.destroyWindow("Goal Calibration")

# For backward compatibility
TrackCalibrator = GoalCalibrator