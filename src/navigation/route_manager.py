# -*- coding: utf-8 -*-
"""
Route-based navigation system that replaces "nearest ball" with fixed route planning
"""

import math

class RouteManager:
    def __init__(self):
        self.route = []  # List of (x, y) coordinates
        self.current_target_index = 0
        self.route_created = False
        self.collected_balls_count = 0
        self.collection_attempts = 0  # Count attempts on current target
        self.max_attempts = 3  # Max attempts before giving up on a ball
        
    def create_route_from_balls(self, balls, robot_center, walls=None, cross_pos=None):
        """Create a fixed route from robot position to all balls with collision avoidance"""
        if self.route_created or not balls:
            return
            
        print("CREATING BALL COLLECTION ROUTE...")
        
        # Filter only balls that are close to cross (walls are OK with perpendicular approach)
        safe_balls = []
        if walls is None:
            walls = []
        
        for ball in balls:
            is_safe = True
            
            # Check only distance to cross (avoid balls closer than 50 cm to cross)
            #if cross_pos:
             #   distance_to_cross = math.sqrt((ball[0] - cross_pos[0])**2 + (ball[1] - cross_pos[1])**2)
              #  if distance_to_cross < 50:  # 50 cm in pixels
               #     print("WARNING: Ball at ({}, {}) too close to cross at ({}, {}) - distance: {:.1f}px".format(
                #        ball[0], ball[1], cross_pos[0], cross_pos[1], distance_to_cross))
                 #   is_safe = False
            
            # Balls near walls are OK - we use perpendicular approach
            if is_safe:
                # Check if ball is close to wall (for info)
                for wall in walls:
                    distance_to_wall = math.sqrt((ball[0] - wall[0])**2 + (ball[1] - wall[1])**2)
                    if distance_to_wall < 50:  # 30 cm in pixels
                        print("Ball at ({}, {}) near wall - will use perpendicular approach".format(ball[0], ball[1]))
                        break
                
                safe_balls.append(ball)
        
        print("Accessible balls: {}/{}".format(len(safe_balls), len(balls)))
        filtered_balls = self.filter_close_balls(safe_balls, min_distance=40)
        print("Filtered balls (no clustering): {}/{}".format(len(filtered_balls), len(safe_balls)))

        # Sort by isolation score (most isolated first)
        filtered_balls.sort(key=lambda b: -self.isolation_score(b, filtered_balls))
        
        if not safe_balls:
            print("No accessible balls found!")
            return
        
        # Start with robot position as starting point
        remaining_balls = list(filtered_balls)  # Copy the list
        route_points = []
        current_pos = robot_center
        
        # Simple "nearest point" route algorithm
        while remaining_balls:
            # Find nearest ball from current position
            distances = [math.sqrt((ball[0] - current_pos[0])**2 + (ball[1] - current_pos[1])**2) 
                        for ball in remaining_balls]
            nearest_index = distances.index(min(distances))
            nearest_ball = remaining_balls.pop(nearest_index)
            
            route_points.append(nearest_ball)
            current_pos = nearest_ball
            
        self.route = route_points
        self.current_target_index = 0
        self.route_created = True
        
        print("ROUTE CREATED: {} waypoints".format(len(self.route)))
        for i, point in enumerate(self.route):
            print("   Point {}: ({}, {})".format(i+1, point[0], point[1]))
            
    def get_current_target(self):
        """Get current target in the route"""
        if not self.route or self.current_target_index >= len(self.route):
            return None
        return self.route[self.current_target_index]
        
    def get_wall_approach_point(self, ball_pos, walls, scale_factor):
        """Calculate optimal approach point for ball near wall (perpendicular approach)"""
        if not walls or scale_factor is None:
            return ball_pos
            
        # Find nearest wall to the ball
        closest_wall = None
        min_distance = float('inf')
        
        for wall in walls:
            distance = math.sqrt((ball_pos[0] - wall[0])**2 + (ball_pos[1] - wall[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_wall = wall
        
        # If ball is closer than 150 px (ca 30 cm) to wall, calculate perpendicular approach
        if closest_wall and min_distance < 150:
            # Calculate vector from wall to ball
            wall_to_ball_x = ball_pos[0] - closest_wall[0]
            wall_to_ball_y = ball_pos[1] - closest_wall[1]
            
            # Normalize vector
            length = math.sqrt(wall_to_ball_x**2 + wall_to_ball_y**2)
            if length > 0:
                norm_x = wall_to_ball_x / length
                norm_y = wall_to_ball_y / length
                
                # Approach point is 50 px (ca 10 cm) behind ball in perpendicular direction from wall
                approach_x = int(ball_pos[0] + norm_x * 50)  # 10 cm behind ball
                approach_y = int(ball_pos[1] + norm_y * 50)
                
                print("WALL APPROACH: Ball at ({}, {}) near wall at ({}, {})".format(
                    ball_pos[0], ball_pos[1], closest_wall[0], closest_wall[1]))
                print("   Approach point: ({}, {}) - 10cm behind ball, perpendicular to wall".format(approach_x, approach_y))
                
                return (approach_x, approach_y)
        
        return ball_pos
        
    def advance_to_next_target(self):
        """Go to next point in the route"""
        self.current_target_index += 1
        self.collected_balls_count += 1
        self.collection_attempts = 0  # Reset attempts for new target
        print("ADVANCING TO NEXT TARGET: {}/{}".format(
            self.current_target_index + 1, len(self.route)))
            
    def increment_collection_attempts(self):
        """Increase number of attempts on current target"""
        self.collection_attempts += 1
        print("Collection attempt {}/{} for current target".format(
            self.collection_attempts, self.max_attempts))
        return self.collection_attempts >= self.max_attempts
        
    def should_skip_current_target(self):
        """Check if we should give up on current target"""
        return self.collection_attempts >= self.max_attempts
        
    def is_route_complete(self):
        """Check if the route is complete"""
        return self.current_target_index >= len(self.route)
        
    def reset_route(self):
        """Reset route for new mission"""
        self.route = []
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0
        print("ROUTE RESET") 

    def filter_close_balls(self, balls, min_distance=40):
        """Remove balls that are too close to each other"""
        filtered = []
        for ball in balls:
            if all(math.dist(ball, b) > min_distance for b in filtered):
                filtered.append(ball)
        return filtered

    def isolation_score(self, ball, others):
        """Return minimum distance to other balls as an isolation score"""
        distances = [math.dist(ball, other) for other in others if other != ball]
        return min(distances) if distances else float('inf')