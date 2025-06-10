"""
Simple goal navigation that:
1. Moves to x=-160, y=0 (first position)
2. Then moves to x=-400, y=0 (goal position)
3. Finally releases balls
"""

import time
from typing import Dict, Optional
from src.camera.coordinate_calculation import calculate_navigation_command
from src.config.settings import *

class GoalNavigation:
    def __init__(self, robot_adapter):
        """robot_adapter should have send_turn_command, send_forward_command, etc."""
        self.robot = robot_adapter
        # Realistic pixel coordinates based on 640x480 camera resolution
        # Assuming robot starts around center (320, 240) and goal is off-screen
        self.first_target_x = 100   # First position - closer to camera edge  
        self.first_target_y = 240   # Keep same Y level as robot
        self.final_target_x = -50   # Goal position - off-screen left
        self.final_target_y = 240   # Keep same Y level
        self.current_state = "MOVING_TO_FIRST"  # States: MOVING_TO_FIRST, MOVING_TO_GOAL, RELEASING_BALLS, DONE
        self.release_start_time = None  # Track when we started releasing balls
        
    def update(self, robot_head: Dict, robot_tail: Dict, scale_factor: Optional[float]) -> bool:
        """
        Update navigation and send commands to robot.
        Returns True if navigation is complete, False otherwise.
        
        Uses the same movement style as vision_app:
        1. Turn to face target
        2. Move forward to target
        """
        if not robot_head or not robot_tail or "pos" not in robot_head or "pos" not in robot_tail:
            return False
            
        # Handle ball release state
        if self.current_state == "RELEASING_BALLS":
            if self.release_start_time is None:
                print("Starting ball release sequence...")
                self.release_start_time = time.time()
                self.robot.release_balls()  # Send release command over network
            elif time.time() - self.release_start_time > 5.0:  # Wait 5 seconds for release to complete
                print("Ball release sequence complete")
                self.current_state = "DONE"
                return True
            return False
            
        # Get current robot position
        head_x = robot_head["pos"][0]
        head_y = robot_head["pos"][1]
        
        # Calculate target position based on current state
        if self.current_state == "MOVING_TO_FIRST":
            target_x = self.first_target_x
            target_y = self.first_target_y
        else:  # MOVING_TO_GOAL
            target_x = self.final_target_x
            target_y = self.final_target_y
            
        # Create a virtual target point for navigation
        target_point = (target_x, target_y)
        
        # Calculate navigation command using the same function as vision_app
        navigation_info = calculate_navigation_command(
            robot_head, robot_tail, target_point, scale_factor
        )
        
        if not navigation_info:
            return False
            
        angle_diff = navigation_info["angle_diff"]
        distance_cm = navigation_info["distance_cm"]
        
        print("Goal Navigation: State={}, Angle diff={:.1f}deg, Distance={:.1f}cm".format(
            self.current_state, angle_diff, distance_cm))
        
        # Check if we've reached the target
        if distance_cm < 20:  # Øget tolerance fra 10 til 20cm for at undgå stuck loops
            if self.current_state == "MOVING_TO_FIRST":
                print("Reached first position! Moving to goal...")
                self.current_state = "MOVING_TO_GOAL"
            elif self.current_state == "MOVING_TO_GOAL":
                print("Reached goal position! Starting ball release sequence...")
                self.current_state = "RELEASING_BALLS"
                self.release_start_time = None  # Will be set in next update
        else:
            # Use same movement logic as vision_app - send commands over network
            if abs(angle_diff) > 10:  # TURN PHASE
                direction = "right" if angle_diff > 0 else "left"
                turn_amount = min(abs(angle_diff), 45)  # Begræns store drejninger for stabilitet
                duration = turn_amount / ESTIMATED_TURN_RATE
                self.robot.send_turn_command(direction, duration)
                print("GOAL NAV: Turning {} {:.1f} degrees".format(direction, turn_amount))
            elif distance_cm > 3:  # FORWARD PHASE
                move_distance = min(distance_cm, MAX_FORWARD_DISTANCE)  # Längere bevægelse for smooth motion
                self.robot.send_forward_command(move_distance)
                print("GOAL NAV: Driving forward {:.1f}cm".format(move_distance))
            else:  # Final approach
                print("GOAL NAV: Final approach")
                self.robot.send_forward_command(5)
                
        return False
    
    def reset(self):
        """Reset navigation state"""
        self.current_state = "MOVING_TO_FIRST"
        self.release_start_time = None
        self.robot.stop_all_motors()


    
    



