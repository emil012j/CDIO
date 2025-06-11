"""
Simpel goal position click-calibration
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
            print("Goal sat til: ({}, {}) - tryk 'q' for at gemme".format(x, y))

    def save_goal(self):
        """Gem goal position"""
        if self.goal_position:
            data = {'calibrated': True, 'goal_position': self.goal_position}
            with open(self.goal_file, 'w') as f:
                json.dump(data, f)
            print("Goal gemt: {}".format(self.goal_position))
            return True
        return False

    def start_calibration(self, frame):
        """Start click calibration"""
        print("Click på goal position, tryk 'q' for at gemme")
        cv2.namedWindow("Goal Calibration")
        cv2.setMouseCallback("Goal Calibration", self.mouse_callback)
        
        display_frame = frame.copy()
        if self.goal_position:
            cv2.circle(display_frame, self.goal_position, 10, (0, 255, 0), -1)
        cv2.putText(display_frame, "Click goal position", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Goal Calibration", display_frame)

# For backward compatibility
TrackCalibrator = GoalCalibrator 