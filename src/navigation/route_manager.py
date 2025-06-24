# -*- coding: utf-8 -*-
"""
Route-baseret navigation system der erstatter "nærmeste bold" med fast rute planlægning
"""

import math
import time
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
        
        # Timer-based timeout for ball collection
        self.target_start_time = None  # When robot started working on current target
        self.max_target_time = 30.0   # Max seconds to spend on one target (30 seconds)
        
    def create_route_from_balls(self, balls, robot_center, cross_pos=None):
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
        
        # Create route using STRICT quadrant rules for ball collection
        route_points = []
        current_pos = robot_center
        remaining_balls = list(filtered_balls)
        
        print("🏠 APPLYING STRICT QUADRANT RULES FOR BALL COLLECTION...")
        
        # Build route with strict quadrant-aware navigation
        while remaining_balls:
            current_quadrant = self.safe_spot_manager.get_robot_quadrant(current_pos)
            
            # First, prioritize balls in the SAME quadrant as robot
            same_quadrant_balls = []
            cross_quadrant_balls = []
            
            for ball in remaining_balls:
                ball_quadrant = self.safe_spot_manager.get_ball_quadrant(ball)
                if ball_quadrant == current_quadrant:
                    same_quadrant_balls.append(ball)
                else:
                    cross_quadrant_balls.append(ball)
            
            target_ball = None
            
            # RULE 1: If there are balls in current quadrant, collect them first
            if same_quadrant_balls:
                print(f"🏠 Robot in Q{current_quadrant}: Found {len(same_quadrant_balls)} balls in same quadrant")
                # Choose closest ball in same quadrant
                min_distance = float('inf')
                for ball in same_quadrant_balls:
                    distance = math.sqrt((ball[0] - current_pos[0])**2 + (ball[1] - current_pos[1])**2)
                    if distance < min_distance:
                        min_distance = distance
                        target_ball = ball

                # Go directly to the ball (no safe spot at start)
                print(f"🎯 Direct route to ball in Q{current_quadrant}")
                route_points.append(target_ball)
                
            # RULE 2: No balls in current quadrant - must go via safe spots to other quadrants
            elif cross_quadrant_balls:
                print(f"🏠 Robot in Q{current_quadrant}: No balls in same quadrant, checking cross-quadrant balls")
                
                # Choose closest cross-quadrant ball
                min_distance = float('inf')
                for ball in cross_quadrant_balls:
                    distance = math.sqrt((ball[0] - current_pos[0])**2 + (ball[1] - current_pos[1])**2)
                    if distance < min_distance:
                        min_distance = distance
                        target_ball = ball
                
                ball_quadrant = self.safe_spot_manager.get_ball_quadrant(target_ball)
                print(f"🚦 Cross-quadrant collection: Q{current_quadrant} → Q{ball_quadrant}")
                
                # ALWAYS route through safe spots for cross-quadrant movement
                # Step 1: Go to current quadrant's safe spot (if not already there)
                current_safe_spot = self.safe_spot_manager.get_safe_spot(current_quadrant)
                is_at_safe_spot = False
                if current_safe_spot:
                    is_at_safe_spot = (abs(current_pos[0] - current_safe_spot[0]) < 20 and 
                                     abs(current_pos[1] - current_safe_spot[1]) < 20)
                
                if not is_at_safe_spot and current_safe_spot:
                    print(f"🛡️ First, navigate to Q{current_quadrant} safe spot")
                    route_points.append(current_safe_spot)
                    # Update current position for next calculation
                    current_pos = current_safe_spot
                
                # Step 2: Get safe route from current quadrant safe spot to target ball
                safe_route = self.safe_spot_manager.plan_safe_route_to_ball(current_pos, target_ball)
                
                # Add all waypoints from safe route
                for waypoint in safe_route:
                    route_points.append(waypoint)
                
            else:
                print("❌ No balls remaining")
                break
            
            if target_ball:
                remaining_balls.remove(target_ball)
                # IMPORTANT: Set current position to the ball's quadrant safe spot, not the ball itself
                # This ensures next cross-quadrant movement starts from safe spot
                ball_quadrant = self.safe_spot_manager.get_ball_quadrant(target_ball)
                ball_safe_spot = self.safe_spot_manager.get_safe_spot(ball_quadrant)
                if ball_safe_spot:
                    current_pos = ball_safe_spot
                    print(f"📍 After collecting ball, positioning at Q{ball_quadrant} safe spot for next navigation")
                else:
                    current_pos = target_ball  # Fallback to ball position
            else:
                break
        
        self.route = route_points
        self.current_target_index = 0
        self.route_created = True
        self.start_target_timer()  # Start timer for first target
        
        print(f"✅ SAFE ROUTE CREATED: {len(self.route)} waypoints")
        for i, point in enumerate(self.route):
            point_type = "WAYPOINT" if self.safe_spot_manager.is_waypoint(point) else "BALL"
            print(f"   {i+1}: {point} ({point_type})")
            
    def get_current_target(self):
        """Få nuværende mål i ruten"""
        print(f"DEBUG: get_current_target called. Route length: {len(self.route)}, Current index: {self.current_target_index}")
        if not self.route:
            print("DEBUG: get_current_target - Route is empty. Returning None.")
            return None
        if self.current_target_index >= len(self.route):
            print(f"DEBUG: get_current_target - current_target_index ({self.current_target_index}) out of bounds for route length ({len(self.route)}). Returning None.")
            return None
        target = self.route[self.current_target_index]
        print(f"DEBUG: get_current_target - Returning target: {target}")
        return target
        
    def advance_to_next_target(self):
        """Gå til næste punkt i ruten"""
        self.current_target_index += 1
        self.collected_balls_count += 1
        self.collection_attempts = 0  # Reset attempts for new target
        self.target_start_time = time.time()  # Reset timer for new target
        print(f"🎯 ADVANCING TO NEXT TARGET: {self.current_target_index + 1}/{len(self.route)} (Route length: {len(self.route)})")
            
    def increment_collection_attempts(self):
        """Øg antal forsøg på nuværende target"""
        self.collection_attempts += 1
        print("⚠️  Collection attempt {}/{} for current target".format(
            self.collection_attempts, self.max_attempts))
        return self.collection_attempts >= self.max_attempts
        
    def start_target_timer(self):
        """Start timer for current target"""
        self.target_start_time = time.time()
        print(f"⏱️ Timer started for target - max {self.max_target_time} seconds")
    
    def check_target_timeout(self):
        """Check if current target has timed out"""
        if self.target_start_time is None:
            self.start_target_timer()
            return False
            
        elapsed_time = time.time() - self.target_start_time
        is_timeout = elapsed_time > self.max_target_time
        
        if is_timeout:
            print(f"⏰ TARGET TIMEOUT: {elapsed_time:.1f}s > {self.max_target_time}s")
        
        return is_timeout
    
    def should_skip_current_target(self):
        """Tjek om vi skal give op på nuværende target (attempts OR timeout)"""
        attempts_exceeded = self.collection_attempts >= self.max_attempts
        timeout_exceeded = self.check_target_timeout()
        
        if attempts_exceeded:
            print(f"⚠️ Skipping target: max attempts ({self.max_attempts}) reached")
        if timeout_exceeded:
            print(f"⏰ Skipping target: timeout ({self.max_target_time}s) exceeded")
            
        return attempts_exceeded or timeout_exceeded
        
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
    
    def create_goal_route(self, robot_center, goal_position, goal_marker=None):
        """Create a safe route to goal using safe spots and (optionally) a marker"""
        if not goal_position:
            print("ERROR: create_goal_route - goal_position is None. Cannot create route.")
            return
            
        print("🎯 CREATING SAFE GOAL ROUTE...")
        
        # Get safe route to goal, passing marker if available
        goal_waypoints = self.safe_spot_manager.plan_safe_route_to_goal(robot_center, goal_position, goal_marker)
        
        self.route = goal_waypoints
        self.current_target_index = 0
        self.route_created = True
        self.start_target_timer()  # Start timer for first target

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