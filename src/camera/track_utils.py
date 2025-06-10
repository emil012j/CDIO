import json
import os
import cv2
from typing import Tuple, Optional

class GoalUtils:
    def __init__(self, goal_file: str = 'goal_positions.json'):
        self.goal_file = goal_file
        self.goal_data = None
        self.load_goal_positions()
    
    def load_goal_positions(self) -> bool:
        """Load goal positions from file"""
        try:
            if self.goal_file and os.path.exists(self.goal_file):
                with open(self.goal_file, 'r') as f:
                    self.goal_data = json.load(f)
                return self.goal_data.get('calibrated', False)
        except Exception as e:
            print("Error loading goal positions: {}".format(e))
        return False
    
    def get_goal_center(self) -> Optional[Tuple[int, int]]:
        """Get the goal center position"""
        if not self.goal_data or not self.goal_data.get('calibrated'):
            return None
        return tuple(self.goal_data['goal_center'])
    
    def get_target_position(self) -> Optional[Tuple[int, int]]:
        """Get the target position (where robot should go)"""
        if not self.goal_data or not self.goal_data.get('calibrated'):
            return None
        return tuple(self.goal_data['target_position'])
    
    def is_goal_calibrated(self) -> bool:
        """Check if goal has been calibrated"""
        return self.goal_data is not None and self.goal_data.get('calibrated', False)
    
    def draw_goal_overlay(self, frame):
        """Draw goal and target positions on frame"""
        if not self.is_goal_calibrated():
            cv2.putText(frame, "Goal Not Calibrated", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return
        
        goal_center = self.get_goal_center()
        target_position = self.get_target_position()
        
        if goal_center:
            cv2.circle(frame, goal_center, 8, (255, 0, 0), -1)
            cv2.putText(frame, "Goal", (goal_center[0]+10, goal_center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        if target_position:
            cv2.circle(frame, target_position, 10, (0, 0, 255), -1)
            cv2.putText(frame, "Target", (target_position[0]+10, target_position[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

# For backward compatibility
TrackUtils = GoalUtils

if __name__ == "__main__":
    goal_utils = GoalUtils()
    
    if not goal_utils.is_goal_calibrated():
        print("No goal calibration found. Please run track_calibration.py first.")
        exit()
    
    print("Goal center:", goal_utils.get_goal_center())
    print("Target position:", goal_utils.get_target_position()) 