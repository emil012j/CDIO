# -*- coding: utf-8 -*-
"""
Route-baseret navigation system der erstatter "nærmeste bold" med fast rute planlægning
"""

import math
from .safe_spot_manager import SafeSpotManager

class RouteManager:
    def __init__(self):
        self.route = []  # Liste af (x, y) koordinater
        self.current_target_index = 0
        self.route_created = False
        self.collected_balls_count = 0
        self.collection_attempts = 0  # Tæl forsøg på nuværende target
        self.max_attempts = 3  # Max forsøg før vi giver op på en bold
        self.safe_spot_manager = SafeSpotManager()  # Safe spot navigation system
        
    def create_route_from_balls(self, balls, robot_center, walls=None, cross_pos=None):
        """Lav en fast rute fra robot position til alle bolde med cross-safe navigation"""
        if self.route_created or not balls:
            return
            
        print("🗺️  CREATING SAFE BALL COLLECTION ROUTE...")
        
        # Filter out balls too close to center cross
        safe_balls = []
        for ball in balls:
            if not self.safe_spot_manager.is_near_center_cross(ball, cross_radius=80):
                safe_balls.append(ball)
            else:
                print(f"⚠️ Skipping ball at {ball} - too close to center cross")
        
        print(f"📍 Safe balls (away from center): {len(safe_balls)}/{len(balls)}")
        
        if not safe_balls:
            print("❌ No safe balls found away from center cross!")
            return
        
        # Prioritize balls by safe access from robot's current quadrant
        prioritized_balls = self.safe_spot_manager.filter_balls_by_safe_access(robot_center, safe_balls)
        
        # Filter close balls to avoid clustering
        filtered_balls = self.filter_close_balls(prioritized_balls, min_distance=40)
        print(f"✅ Final balls (no clustering): {len(filtered_balls)}/{len(prioritized_balls)}")
        
        if not filtered_balls:
            print("❌ No accessible balls found after filtering!")
            return
        
        # Create route using safe spot navigation
        route_points = []
        current_pos = robot_center
        remaining_balls = list(filtered_balls)
        
        # Build route with safe navigation
        while remaining_balls:
            # Find best next ball considering safe paths
            best_ball = None
            best_distance = float('inf')
            best_route = None
            
            for ball in remaining_balls:
                # Get safe route to this ball
                safe_route = self.safe_spot_manager.plan_safe_route_to_ball(current_pos, ball)
                
                # Calculate total route distance
                total_distance = 0
                route_pos = current_pos
                for waypoint in safe_route:
                    total_distance += math.sqrt((waypoint[0] - route_pos[0])**2 + (waypoint[1] - route_pos[1])**2)
                    route_pos = waypoint
                
                if total_distance < best_distance:
                    best_distance = total_distance
                    best_ball = ball
                    best_route = safe_route
            
            if best_ball:
                # Add waypoints for this ball (excluding the ball itself for now)
                for waypoint in best_route[:-1]:  # All except the ball
                    route_points.append(waypoint)
                
                # Add the ball as final waypoint
                route_points.append(best_ball)
                
                remaining_balls.remove(best_ball)
                current_pos = best_ball
            else:
                break
        
        self.route = route_points
        self.current_target_index = 0
        self.route_created = True
        
        print(f"✅ SAFE ROUTE CREATED: {len(self.route)} waypoints")
        for i, point in enumerate(self.route):
            point_type = "WAYPOINT" if self.safe_spot_manager.is_waypoint(point) else "BALL"
            print(f"   {i+1}: {point} ({point_type})")
            
    def get_current_target(self):
        """Få nuværende mål i ruten"""
        if not self.route or self.current_target_index >= len(self.route):
            return None
        return self.route[self.current_target_index]
        
    def get_wall_approach_point(self, ball_pos, walls, scale_factor):
        """Beregn optimal tilgangspunkt for bold tæt på væg (vinkelret tilgang)"""
        if not walls or scale_factor is None:
            return ball_pos
            
        # Find nærmeste væg til bolden
        closest_wall = None
        min_distance = float('inf')
        
        for wall in walls:
            distance = math.sqrt((ball_pos[0] - wall[0])**2 + (ball_pos[1] - wall[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_wall = wall
        
        # Hvis bold er tættere end 150 px (ca 30 cm) til væg, beregn vinkelret tilgang
        if closest_wall and min_distance < 150:
            # Beregn vektor fra væg til bold
            wall_to_ball_x = ball_pos[0] - closest_wall[0]
            wall_to_ball_y = ball_pos[1] - closest_wall[1]
            
            # Normaliser vektor
            length = math.sqrt(wall_to_ball_x**2 + wall_to_ball_y**2)
            if length > 0:
                norm_x = wall_to_ball_x / length
                norm_y = wall_to_ball_y / length
                
                # Tilgangspunkt er 50 px (ca 10 cm) bag bolden i vinkelret retning fra væg
                approach_x = int(ball_pos[0] + norm_x * 50)  # 10 cm bag bolden
                approach_y = int(ball_pos[1] + norm_y * 50)
                
                print("🧱 WALL APPROACH: Ball at ({}, {}) near wall at ({}, {})".format(
                    ball_pos[0], ball_pos[1], closest_wall[0], closest_wall[1]))
                print("   → Approach point: ({}, {}) - 10cm behind ball, perpendicular to wall".format(approach_x, approach_y))
                
                return (approach_x, approach_y)
        
        return ball_pos
        
    def advance_to_next_target(self):
        """Gå til næste punkt i ruten"""
        self.current_target_index += 1
        self.collected_balls_count += 1
        self.collection_attempts = 0  # Reset attempts for new target
        print("🎯 ADVANCING TO NEXT TARGET: {}/{}".format(
            self.current_target_index + 1, len(self.route)))
            
    def increment_collection_attempts(self):
        """Øg antal forsøg på nuværende target"""
        self.collection_attempts += 1
        print("⚠️  Collection attempt {}/{} for current target".format(
            self.collection_attempts, self.max_attempts))
        return self.collection_attempts >= self.max_attempts
        
    def should_skip_current_target(self):
        """Tjek om vi skal give op på nuværende target"""
        return self.collection_attempts >= self.max_attempts
        
    def is_route_complete(self):
        """Tjek om ruten er færdig"""
        return self.current_target_index >= len(self.route)
        
    def reset_route(self):
        """Reset rute for ny mission"""
        self.route = []
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0
        print("🔄 ROUTE RESET") 

    def create_goal_route(self, robot_center, goal_position):
        """Create a safe route to goal using safe spots"""
        if not goal_position:
            return
            
        print("🎯 CREATING SAFE GOAL ROUTE...")
        
        # Get safe route to goal
        goal_waypoints = self.safe_spot_manager.plan_safe_route_to_goal(robot_center, goal_position)
        
        self.route = goal_waypoints
        self.current_target_index = 0
        self.route_created = True
        
        print(f"✅ SAFE GOAL ROUTE CREATED: {len(self.route)} waypoints")
        for i, point in enumerate(self.route):
            point_type = "WAYPOINT" if self.safe_spot_manager.is_waypoint(point) else "GOAL"
            print(f"   {i+1}: {point} ({point_type})")

    def is_current_target_waypoint(self):
        """Check if current target is a waypoint (not ball/goal)"""
        current_target = self.get_current_target()
        if current_target:
            return self.safe_spot_manager.is_waypoint(current_target)
        return False

    def filter_close_balls(self, balls, min_distance=40):
        """Fjern bolde der er for tæt på hinanden"""
        filtered = []
        for ball in balls:
            if all(math.dist(ball, b) > min_distance for b in filtered):
                filtered.append(ball)
        return filtered

    def isolation_score(self, ball, others):
        """Returner minimumsafstand til andre bolde som et isolationsmål"""
        distances = [math.dist(ball, other) for other in others if other != ball]
        return min(distances) if distances else float('inf')