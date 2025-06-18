"""
Simpel goal position loader og display med delivery og goal punkter
"""

import json
import os
import cv2
import math

class GoalUtils:
    def __init__(self):
        self.goal_file = 'goal_positions.json'
        self.delivery_position = None
        self.goal_position = None
        self.load_goal()
    
    def load_goal(self):
        """Load delivery og goal positioner fra fil"""
        try:
            if os.path.exists(self.goal_file):
                with open(self.goal_file, 'r') as f:
                    data = json.load(f)
                    if data.get('calibrated'):
                        # Load both positions if available
                        if 'delivery_position' in data:
                            self.delivery_position = tuple(data['delivery_position'])
                        if 'goal_position' in data:
                            self.goal_position = tuple(data['goal_position'])
                        return True
        except:
            pass
        return False
    
    def get_delivery_position(self):
        """Få delivery position"""
        return self.delivery_position
    
    def get_goal_position(self):
        """Få goal position (backward compatibility)"""
        return self.goal_position
    
    def get_primary_goal_position(self):
        """Få den primære goal position - hvis delivery er sat, brug den, ellers goal"""
        if self.delivery_position:
            return self.delivery_position
        return self.goal_position
    
    def draw_goal_on_frame(self, frame):
        """Tegn delivery og goal på frame"""
        # Tegn delivery punkt (gul)
        if self.delivery_position:
            cv2.circle(frame, self.delivery_position, 10, (0, 255, 255), -1)
            cv2.putText(frame, "DELIVERY", (self.delivery_position[0]+15, self.delivery_position[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Tegn goal punkt (grøn)
        if self.goal_position:
            cv2.circle(frame, self.goal_position, 10, (0, 255, 0), -1)
            cv2.putText(frame, "GOAL", (self.goal_position[0]+15, self.goal_position[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Tegn linje mellem delivery og goal hvis begge er sat
        if self.delivery_position and self.goal_position:
            cv2.line(frame, self.delivery_position, self.goal_position, (255, 255, 255), 2)
            # Tegn vinkelret tilgang indikation
            mid_x = (self.delivery_position[0] + self.goal_position[0]) // 2
            mid_y = (self.delivery_position[1] + self.goal_position[1]) // 2
            cv2.putText(frame, "PERPENDICULAR", (mid_x-50, mid_y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# For backward compatibility
TrackUtils = GoalUtils 