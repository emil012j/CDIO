"""
Simple goal position loader and display
"""

import json
import os
import cv2

class GoalUtils:
    def __init__(self):
        self.goal_file = 'goal_positions.json'
        self.goal_position = None
        self.load_goal()
    
    def load_goal(self):
        """Load goal position from file"""
        try:
            if os.path.exists(self.goal_file):
                with open(self.goal_file, 'r') as f:
                    data = json.load(f)
                    if data.get('calibrated'):
                        self.goal_position = tuple(data['goal_position'])
                        return True
        except:
            pass
        return False
    
    def get_goal_position(self):
        """Get goal position"""
        return self.goal_position
    
    def draw_goal_on_frame(self, frame):
        """Draw goal on frame"""
        if self.goal_position:
            cv2.circle(frame, self.goal_position, 10, (255, 0, 0), -1)
            cv2.putText(frame, "GOAL", (self.goal_position[0]+15, self.goal_position[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

# For backward compatibility
TrackUtils = GoalUtils 