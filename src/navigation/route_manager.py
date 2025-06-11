# -*- coding: utf-8 -*-
"""
Route-baseret navigation system der erstatter "nærmeste bold" med fast rute planlægning
"""

import math

class RouteManager:
    def __init__(self):
        self.route = []  # Liste af (x, y) koordinater
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0  # Tæl forsøg på nuværende target
        self.max_attempts = 3  # Max forsøg før vi giver op på en bold
        
    def create_route_from_balls(self, balls, robot_center, walls=None, cross_pos=None):
        """Lav en fast rute fra robot position til alle bolde med kollisionsundgåelse"""
        if self.route_created or not balls:
            return
            
        print("🗺️  CREATING BALL COLLECTION ROUTE...")
        
        # Filtrer kun bolde der er for tæt på kors (vægge er OK med vinkelret tilgang)
        safe_balls = []
        if walls is None:
            walls = []
        
        for ball in balls:
            is_safe = True
            
            # Tjek kun afstand til kors (undgå bolde tættere end 50 cm til kors)
            if cross_pos:
                distance_to_cross = math.sqrt((ball[0] - cross_pos[0])**2 + (ball[1] - cross_pos[1])**2)
                if distance_to_cross < 250:  # 50 cm i pixels
                    print("⚠️  Ball at ({}, {}) too close to cross at ({}, {}) - distance: {:.1f}px".format(
                        ball[0], ball[1], cross_pos[0], cross_pos[1], distance_to_cross))
                    is_safe = False
            
            # Bolde tæt på vægge er OK - vi bruger vinkelret tilgang
            if is_safe:
                # Tjek om bold er tæt på væg (for info)
                for wall in walls:
                    distance_to_wall = math.sqrt((ball[0] - wall[0])**2 + (ball[1] - wall[1])**2)
                    if distance_to_wall < 150:  # 30 cm i pixels
                        print("🧱 Ball at ({}, {}) near wall - will use perpendicular approach".format(ball[0], ball[1]))
                        break
                
                safe_balls.append(ball)
        
        print("📍 Accessible balls: {}/{}".format(len(safe_balls), len(balls)))
        
        if not safe_balls:
            print("❌ No accessible balls found!")
            return
        
        # Start med robot position som udgangspunkt
        remaining_balls = list(safe_balls)  # Kopier listen
        route_points = []
        current_pos = robot_center
        
        # Simpel "nærmeste punkt" rute algoritme
        while remaining_balls:
            # Find nærmeste bold fra nuværende position
            distances = [math.sqrt((ball[0] - current_pos[0])**2 + (ball[1] - current_pos[1])**2) 
                        for ball in remaining_balls]
            nearest_index = distances.index(min(distances))
            nearest_ball = remaining_balls.pop(nearest_index)
            
            route_points.append(nearest_ball)
            current_pos = nearest_ball
            
        self.route = route_points
        self.current_target_index = 0
        self.route_created = True
        
        print("✅ ROUTE CREATED: {} waypoints".format(len(self.route)))
        for i, point in enumerate(self.route):
            print("   Point {}: ({}, {})".format(i+1, point[0], point[1]))
            
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