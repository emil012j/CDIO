# -*- coding: utf-8 -*-
"""
Safe Spot Navigation System - 4 quadrant safe spots to avoid crossing center
"""

import math

class SafeSpotManager:
    def __init__(self, camera_resolution=(1280, 720)):
        self.camera_width, self.camera_height = camera_resolution
        self.center_x = self.camera_width // 2
        self.center_y = self.camera_height // 2
        
        # Define 4 safe spots in each quadrant (away from center cross)
        # These are positioned ~30% from edges to avoid walls and center
        margin_x = int(self.camera_width * 0.3)
        margin_y = int(self.camera_height * 0.3)
        
        self.safe_spots = {
            1: (self.center_x - margin_x, self.center_y - margin_y),  # Top-left quadrant
            2: (self.center_x + margin_x, self.center_y - margin_y),  # Top-right quadrant  
            3: (self.center_x - margin_x, self.center_y + margin_y),  # Bottom-left quadrant
            4: (self.center_x + margin_x, self.center_y + margin_y)   # Bottom-right quadrant
        }
        
        # Define which quadrants can connect (avoid crossing center)
        self.allowed_connections = {
            1: [1, 2, 3],  # Can go to adjacent quadrants, not diagonal
            2: [1, 2, 4],
            3: [1, 3, 4], 
            4: [2, 3, 4]
        }
        
        print("🛡️ Safe Spot Manager initialized:")
        for spot_id, pos in self.safe_spots.items():
            print(f"   Safe Spot {spot_id}: {pos}")
    
    def get_robot_quadrant(self, robot_pos):
        """Determine which quadrant the robot is currently in"""
        x, y = robot_pos
        if x < self.center_x and y < self.center_y:
            return 1  # Top-left
        elif x >= self.center_x and y < self.center_y:
            return 2  # Top-right
        elif x < self.center_x and y >= self.center_y:
            return 3  # Bottom-left
        else:
            return 4  # Bottom-right
    
    def get_ball_quadrant(self, ball_pos):
        """Determine which quadrant a ball is in"""
        return self.get_robot_quadrant(ball_pos)
    
    def is_cross_safe_path(self, from_quadrant, to_quadrant):
        """Check if path between quadrants is safe (doesn't cross center)"""
        return to_quadrant in self.allowed_connections[from_quadrant]
    
    def get_safe_spot(self, quadrant):
        """Get the safe spot position for a given quadrant"""
        return self.safe_spots.get(quadrant)
    
    def get_nearest_safe_spot(self, position):
        """Get the nearest safe spot to a position"""
        min_distance = float('inf')
        nearest_spot = None
        nearest_quadrant = None
        
        for quadrant, spot_pos in self.safe_spots.items():
            distance = math.sqrt((position[0] - spot_pos[0])**2 + (position[1] - spot_pos[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_spot = spot_pos
                nearest_quadrant = quadrant
                
        return nearest_quadrant, nearest_spot
    
    def plan_safe_route_to_ball(self, robot_pos, ball_pos):
        """Plan a safe route from robot to ball using safe spots if needed"""
        robot_quadrant = self.get_robot_quadrant(robot_pos)
        ball_quadrant = self.get_ball_quadrant(ball_pos)
        
        print(f"🗺️ Planning route: Robot in Q{robot_quadrant} → Ball in Q{ball_quadrant}")
        
        # If ball is in same quadrant or safely accessible, go direct
        if self.is_cross_safe_path(robot_quadrant, ball_quadrant):
            print(f"✅ Direct path safe: Q{robot_quadrant} → Q{ball_quadrant}")
            return [ball_pos]  # Direct route
        
        # Need to go via safe spots to avoid crossing center
        print(f"⚠️ Need safe route via waypoints (cannot go Q{robot_quadrant} → Q{ball_quadrant} directly)")
        
        # Find intermediate quadrant(s) to reach target safely
        route_waypoints = []
        
        # Strategy: Go through adjacent quadrants
        if robot_quadrant == 1 and ball_quadrant == 4:
            # Q1 → Q4: Go via Q2 or Q3
            route_waypoints = [self.safe_spots[2], ball_pos]  # Via top-right
        elif robot_quadrant == 2 and ball_quadrant == 3:
            # Q2 → Q3: Go via Q1 or Q4  
            route_waypoints = [self.safe_spots[4], ball_pos]  # Via bottom-right
        elif robot_quadrant == 3 and ball_quadrant == 2:
            # Q3 → Q2: Go via Q1 or Q4
            route_waypoints = [self.safe_spots[1], ball_pos]  # Via top-left
        elif robot_quadrant == 4 and ball_quadrant == 1:
            # Q4 → Q1: Go via Q2 or Q3
            route_waypoints = [self.safe_spots[3], ball_pos]  # Via bottom-left
        else:
            # Fallback: Go to nearest safe spot then to ball
            _, nearest_spot = self.get_nearest_safe_spot(ball_pos)
            route_waypoints = [nearest_spot, ball_pos]
        
        print(f"🛡️ Safe route planned: {len(route_waypoints)} waypoints")
        for i, waypoint in enumerate(route_waypoints):
            print(f"   Waypoint {i+1}: {waypoint}")
            
        return route_waypoints
    
    def filter_balls_by_safe_access(self, robot_pos, balls):
        """Filter balls to prioritize those with safe access paths"""
        robot_quadrant = self.get_robot_quadrant(robot_pos)
        
        safe_balls = []
        unsafe_balls = []
        
        for ball in balls:
            ball_quadrant = self.get_ball_quadrant(ball)
            if self.is_cross_safe_path(robot_quadrant, ball_quadrant):
                safe_balls.append(ball)
            else:
                unsafe_balls.append(ball)
        
        print(f"🛡️ Ball accessibility from Q{robot_quadrant}:")
        print(f"   Safe access: {len(safe_balls)} balls")
        print(f"   Needs waypoints: {len(unsafe_balls)} balls")
        
        # Return safe balls first, then unsafe ones
        return safe_balls + unsafe_balls
    
    def is_near_center_cross(self, position, cross_radius=100):
        """Check if position is too close to center cross"""
        distance_to_center = math.sqrt((position[0] - self.center_x)**2 + (position[1] - self.center_y)**2)
        return distance_to_center < cross_radius
    
    def draw_safe_spots(self, frame):
        """Draw safe spots on the display frame for visualization"""
        import cv2
        
        # Draw quadrant boundaries
        cv2.line(frame, (self.center_x, 0), (self.center_x, self.camera_height), (128, 128, 128), 1)  # Vertical
        cv2.line(frame, (0, self.center_y), (self.camera_width, self.center_y), (128, 128, 128), 1)   # Horizontal
        
        # Draw safe spots
        for quadrant, spot_pos in self.safe_spots.items():
            # Large green circle for safe spot
            cv2.circle(frame, spot_pos, 20, (0, 255, 0), 3)
            cv2.circle(frame, spot_pos, 15, (0, 255, 0), -1)
            
            # Label
            cv2.putText(frame, f"SAFE {quadrant}", (spot_pos[0] - 30, spot_pos[1] - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw center danger zone
        cv2.circle(frame, (self.center_x, self.center_y), 100, (0, 0, 255), 2)
        cv2.putText(frame, "DANGER ZONE", (self.center_x - 50, self.center_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2) 