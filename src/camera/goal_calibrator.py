"""
Simple goal position click-calibration
"""

import cv2
import json

class GoalCalibrator:
    def __init__(self):
        self.goal_file = "goal_positions.json"
        self.goal_position = None
        
    def mouse_callback(self, event, x, y, flags, param):
        """Click to set goal position"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.goal_position = (x, y)
            print("Goal set to: ({}, {}) - press 'q' to save".format(x, y))

    def save_goal(self):
        """Save goal position"""
        if self.goal_position:
            # Convert tuple to list for JSON serialization
            goal_list = [self.goal_position[0], self.goal_position[1]]
            data = {'calibrated': True, 'goal_position': goal_list}
            with open(self.goal_file, 'w') as f:
                json.dump(data, f, indent=2)  # Pretty format for readability
            print("Goal saved: {} -> {}".format(self.goal_position, self.goal_file))
            return True
        return False

    def start_calibration(self, frame):
        """Start click calibration with proper event loop"""
        print("Click on goal position, press 'q' to save, 'ESC' to cancel")
        cv2.namedWindow("Goal Calibration")
        cv2.setMouseCallback("Goal Calibration", self.mouse_callback)
        
        # Calibration loop
        while True:
            display_frame = frame.copy()
            
            # Draw current goal position if set
            if self.goal_position:
                cv2.circle(display_frame, self.goal_position, 15, (0, 255, 0), -1)
                cv2.putText(display_frame, "GOAL: ({},{})".format(self.goal_position[0], self.goal_position[1]), 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame, "Press 'q' to save, ESC to cancel", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                cv2.putText(display_frame, "Click to set goal position", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, "ESC to cancel", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_frame, "Goal Calibration", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Goal Calibration", display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Save goal
                if self.goal_position:
                    if self.save_goal():
                        print("Goal successfully saved!")
                        break
                    else:
                        print("Failed to save goal!")
                else:
                    print("No goal position set - click on the image first!")
            elif key == 27:  # ESC key - cancel
                print("Goal calibration cancelled")
                break
        
        cv2.destroyWindow("Goal Calibration")

# For backward compatibility
TrackCalibrator = GoalCalibrator 