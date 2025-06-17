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
        self.collected_balls_count = 0
        self.collection_attempts = 0
        self.max_attempts = 3  # Max forsøg før vi giver op på en bold

    def create_route_from_balls(self, balls, robot_center, walls=None, cross_pos=None, min_cross_distance=50):
        """Lav en fast rute fra robot position til alle bolde med kollisionsundgåelse"""
        if self.route_created or not balls:
            return

        print("  CREATING BALL COLLECTION ROUTE...")

        safe_balls = []
        if walls is None:
            walls = []

        for ball in balls:
            is_safe = True

            # Undgå bolde for tæt på kors
            if cross_pos:
                dx = ball[0] - cross_pos[0]
                dy = ball[1] - cross_pos[1]
                distance_to_cross = math.sqrt(dx**2 + dy**2)
                if distance_to_cross < min_cross_distance:
                    print(" Ball at ({}, {}) is {:.1f}px from cross ({}, {}) (too close)".format(
                        ball[0], ball[1], distance_to_cross, cross_pos[0], cross_pos[1]))
                    is_safe = False

            if is_safe:
                for wall in walls:
                    distance_to_wall = math.hypot(ball[0] - wall[0], ball[1] - wall[1])
                    if distance_to_wall < 150:
                        print(" Ball at ({}, {}) near wall - will use perpendicular approach".format(
                            ball[0], ball[1]))
                        break
                safe_balls.append(ball)

        print(" Accessible balls: {}/{}".format(len(safe_balls), len(balls)))
        if not safe_balls:
            print(" No accessible balls found!")
            return

        remaining_balls = list(safe_balls)
        route_points = []
        current_pos = robot_center

        while remaining_balls:
            distances = [math.hypot(ball[0] - current_pos[0], ball[1] - current_pos[1]) for ball in remaining_balls]
            nearest_index = distances.index(min(distances))
            nearest_ball = remaining_balls.pop(nearest_index)
            route_points.append(nearest_ball)
            current_pos = nearest_ball

        self.route = route_points
        self.current_target_index = 0
        self.route_created = True

        print(" ROUTE CREATED: {} waypoints".format(len(self.route)))
        for i, point in enumerate(self.route):
            print("   Point {}: ({}, {})".format(i + 1, point[0], point[1]))

    def get_current_target(self):
        if not self.route or self.current_target_index >= len(self.route):
            return None
        return self.route[self.current_target_index]

    def get_wall_approach_point(self, ball_pos, walls, scale_factor):
        if not walls or scale_factor is None:
            return ball_pos

        closest_wall = None
        min_distance = float('inf')

        for wall in walls:
            distance = math.hypot(ball_pos[0] - wall[0], ball_pos[1] - wall[1])
            if distance < min_distance:
                min_distance = distance
                closest_wall = wall

        if closest_wall and min_distance < 150:
            dx = ball_pos[0] - closest_wall[0]
            dy = ball_pos[1] - closest_wall[1]
            length = math.hypot(dx, dy)
            if length > 0:
                norm_x = dx / length
                norm_y = dy / length
                approach_x = int(ball_pos[0] + norm_x * 50)
                approach_y = int(ball_pos[1] + norm_y * 50)
                print(" WALL APPROACH: Ball at ({}, {}) near wall at ({}, {})".format(
                    ball_pos[0], ball_pos[1], closest_wall[0], closest_wall[1]))
                print("   → Approach point: ({}, {})".format(approach_x, approach_y))
                return (approach_x, approach_y)

        return ball_pos

    def advance_to_next_target(self):
        self.current_target_index += 1
        self.collected_balls_count += 1
        self.collection_attempts = 0
        print(" ADVANCING TO NEXT TARGET: {}/{}".format(
            self.current_target_index + 1, len(self.route)))

    def increment_collection_attempts(self):
        self.collection_attempts += 1
        print("  Collection attempt {}/{} for current target".format(
            self.collection_attempts, self.max_attempts))
        return self.collection_attempts >= self.max_attempts

    def should_skip_current_target(self):
        return self.collection_attempts >= self.max_attempts

    def is_route_complete(self):
        return self.current_target_index >= len(self.route)

    def reset_route(self):
        self.route = []
        self.current_target_index = 0
        self.route_created = False
        self.collection_attempts = 0
        print(" ROUTE RESET")
