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
        margin_x = int(self.camera_width * 0.2)
        margin_y = int(self.camera_height * 0.2)
        
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

        route_waypoints = []

        # Step 1: Go to own safe spot if not already there
        own_safe_spot = self.safe_spots[robot_quadrant]
        if math.dist(robot_pos, own_safe_spot) > 50:
            print(f"🛡️ Adding own safe spot: {own_safe_spot}")
            route_waypoints.append(own_safe_spot)

        # Step 2: If not already in target quadrant, go to its safe spot
        if robot_quadrant != ball_quadrant:
            target_safe_spot = self.safe_spots[ball_quadrant]
            print(f"🛡️ Adding target quadrant safe spot: {target_safe_spot}")
            route_waypoints.append(target_safe_spot)

        # Step 3: Go to the ball
        route_waypoints.append(ball_pos)

        print(f"🛡️ Safe route planned: {len(route_waypoints)} waypoints")
        for i, waypoint in enumerate(route_waypoints):
            print(f"   Waypoint {i+1}: {waypoint}")

        return route_waypoints
    
    def get_two_nearest_safe_spots_to_goal(self, goal_pos):
        """Get the 2 nearest safe spots to the goal position"""
        distances = []
        for quadrant, spot_pos in self.safe_spots.items():
            distance = math.sqrt((goal_pos[0] - spot_pos[0])**2 + (goal_pos[1] - spot_pos[1])**2)
            distances.append((distance, quadrant, spot_pos))
        
        # Sort by distance and get the 2 closest
        distances.sort()
        nearest_spot = distances[0][1], distances[0][2]  # (quadrant, position)
        second_nearest_spot = distances[1][1], distances[1][2]  # (quadrant, position)
        
        return nearest_spot, second_nearest_spot
    
    def calculate_perpendicular_approach_point(self, goal_pos):
        """Calculate perpendicular approach point between 2 nearest safe spots to goal"""
        nearest_spot, second_nearest_spot = self.get_two_nearest_safe_spots_to_goal(goal_pos)
        
        # Calculate midpoint between the 2 nearest safe spots
        nearest_pos = nearest_spot[1]
        second_nearest_pos = second_nearest_spot[1]
        
        midpoint_x = (nearest_pos[0] + second_nearest_pos[0]) // 2
        midpoint_y = (nearest_pos[1] + second_nearest_pos[1]) // 2
        perpendicular_approach = (midpoint_x, midpoint_y)
        
        print(f"🎯 Perpendicular approach calculation:")
        print(f"   Nearest safe spots to goal: Q{nearest_spot[0]} {nearest_pos} and Q{second_nearest_spot[0]} {second_nearest_pos}")
        print(f"   Perpendicular approach point: {perpendicular_approach}")
        
        return perpendicular_approach, nearest_spot[0], second_nearest_spot[0]

    def plan_safe_route_to_goal(self, robot_pos, goal_pos):
        """Plan a safe route from robot to goal using safe spots"""
        robot_quadrant = self.get_robot_quadrant(robot_pos)
        goal_quadrant = self.get_robot_quadrant(goal_pos)

        print(f"🎯 Planning safe route to goal: Robot in Q{robot_quadrant} → Goal in Q{goal_quadrant}")

        route_waypoints = []

        # Step 1: Go to own safe spot if not already there
        own_safe_spot = self.safe_spots[robot_quadrant]
        if math.dist(robot_pos, own_safe_spot) > 50:
            print(f"🛡️ Adding own safe spot: {own_safe_spot}")
            route_waypoints.append(own_safe_spot)

        # Step 2: If not already in goal quadrant, go to its safe spot
        if robot_quadrant != goal_quadrant:
            goal_safe_spot = self.safe_spots[goal_quadrant]
            print(f"🛡️ Adding goal quadrant safe spot: {goal_safe_spot}")
            route_waypoints.append(goal_safe_spot)

        # Step 3: Go to the goal
        route_waypoints.append(goal_pos)

        print(f"🛡️ Safe goal route planned: {len(route_waypoints)} waypoints")
        for i, waypoint in enumerate(route_waypoints):
            print(f"   Waypoint {i+1}: {waypoint}")

        return route_waypoints
    
    def is_waypoint(self, position):
        """Check if a position is a waypoint (safe spot or perpendicular approach point)"""
        # Check if it's a safe spot
        for spot_pos in self.safe_spots.values():
            if position == spot_pos:
                return True
        
        # Check if it might be a perpendicular approach point (between two safe spots)
        # We can check if it's roughly between any two safe spots
        safe_positions = list(self.safe_spots.values())
        for i in range(len(safe_positions)):
            for j in range(i + 1, len(safe_positions)):
                pos1 = safe_positions[i]
                pos2 = safe_positions[j]
                
                # Calculate midpoint between these two safe spots
                midpoint_x = (pos1[0] + pos2[0]) // 2
                midpoint_y = (pos1[1] + pos2[1]) // 2
                
                # Check if position is close to this midpoint (within 20 pixels)
                if abs(position[0] - midpoint_x) < 20 and abs(position[1] - midpoint_y) < 20:
                    return True
        
        return False
    
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
        
 