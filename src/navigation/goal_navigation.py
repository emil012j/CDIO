"""
Simple goal navigation that:
1. Moves to x=-160, y=0 (first position)
2. Then moves to x=-400, y=0 (goal position)
3. Finally releases balls
"""

from typing import Dict
from src.robot.movement import execute_coordinate_command
from src.robot.controller import RobotController

class GoalNavigation:
    def __init__(self, robot: RobotController):
        self.robot = robot
        self.first_target_x = -160  # First position
        self.final_target_x = -400  # Goal position
        self.current_state = "MOVING_TO_FIRST"  # States: MOVING_TO_FIRST, MOVING_TO_GOAL, RELEASING_BALLS, DONE
        
    def update(self, robot_head: Dict, robot_tail: Dict) -> bool:
        """
        Update navigation and send commands to robot.
        Returns True if navigation is complete, False otherwise.
        """
        if not robot_head or not robot_tail or "pos" not in robot_head or "pos" not in robot_tail:
            return False
            
        # Get current robot head position
        head_x = robot_head["pos"][0]
        head_y = robot_head["pos"][1]
        
        if self.current_state == "MOVING_TO_FIRST":
            # Check if we've reached first position
            if abs(head_x - self.first_target_x) < 10 and abs(head_y) < 10:
                print("Reached first position! Moving to goal...")
                self.current_state = "MOVING_TO_GOAL"
            else:
                # Navigate to first position
                command = {
                    "coordinates": {
                        "current_x": head_x,
                        "current_y": head_y,
                        "target_x": self.first_target_x,
                        "target_y": 0  # Always target y=0
                    }
                }
                execute_coordinate_command(self.robot, command)
                
        elif self.current_state == "MOVING_TO_GOAL":
            # Check if we've reached goal position
            if abs(head_x - self.final_target_x) < 10 and abs(head_y) < 10:
                print("Reached goal position! Releasing balls...")
                self.current_state = "RELEASING_BALLS"
                self.robot.release_balls()
                self.current_state = "DONE"
                return True
            else:
                # Navigate to goal position
                command = {
                    "coordinates": {
                        "current_x": head_x,
                        "current_y": head_y,
                        "target_x": self.final_target_x,
                        "target_y": 0  # Always target y=0
                    }
                }
                execute_coordinate_command(self.robot, command)
                
        return False
    
    def reset(self):
        """Reset navigation state"""
        self.current_state = "MOVING_TO_FIRST"
        self.robot.stop_all_motors()


    
    



