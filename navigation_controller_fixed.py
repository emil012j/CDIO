#!/usr/bin/env python3

import math
import time
from controllers.motor_controller import MotorController
from controllers.pickup_controller import PickupController

class NavigationController:
    """Controls the robot's navigation system based on detected coordinates"""
    
    def __init__(self):
        """Initialize the navigation controller"""
        self.motor_controller = MotorController()
        self.pickup_controller = PickupController()
        
        # Navigation parameters
        self.target_precision = 20  # How close we need to be to count as "arrived" (in pixels/mm)
        self.min_distance = 50     # Minimum distance to consider for movement
        self.avoid_distance = 100  # Distance to maintain from obstacles
        
        # Navigation state
        self.current_target = None
        self.current_obstacles = []
        self.robot_pos = None
        self.robot_angle = 0
        
        # Behavior settings
        self.collect_white_balls = True
        self.collect_orange_balls = True
        self.avoid_eggs = True
        
        self.initialized = self.motor_controller.initialized
        print("Navigation controller initialized successfully" if self.initialized else "Navigation controller initialization failed")
    
    def set_robot_position(self, head_pos, tail_pos):
        """Update the robot's position based on head and tail coordinates"""
        if head_pos is None or tail_pos is None:
            self.robot_pos = None
            self.robot_angle = 0
            return False
            
        # Calculate center position
        x1, y1 = head_pos
        x2, y2 = tail_pos
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        # Calculate orientation angle (in degrees)
        dx = x1 - x2
        dy = y1 - y2
        angle = math.degrees(math.atan2(dy, dx))
        
        # Update state
        self.robot_pos = (center_x, center_y)
        self.robot_angle = angle
        return True
    
    def update_from_camera_data(self, camera_data):
        """Update navigation state based on camera data"""
        try:
            # Update robot position
            robot_data = camera_data.get("robot", {})
            head_pos = robot_data.get("head")
            tail_pos = robot_data.get("tail")
            
            if head_pos and tail_pos:
                self.set_robot_position(head_pos, tail_pos)
            else:
                self.robot_pos = None
                return False
                
            # Update orientation if provided directly
            if robot_data.get("orientation") is not None:
                self.robot_angle = robot_data["orientation"]
                
            # Update target list (balls)
            self.current_target = None
            targets = camera_data.get("targets", [])
            
            white_balls = []
            orange_balls = []
            
            for target in targets:
                if target["type"] == "white ball" and self.collect_white_balls:
                    white_balls.append(target)
                elif target["type"] == "orange ball" and self.collect_orange_balls:
                    orange_balls.append(target)
            
            # Prioritize targets (you can change this logic based on your needs)
            all_balls = white_balls + orange_balls
            if all_balls:
                # Find closest ball
                closest_ball = min(all_balls, key=lambda b: self._calculate_distance(
                    self.robot_pos, b["position"]))
                self.current_target = closest_ball
            
            # Update obstacles list
            self.current_obstacles = []
            obstacles = camera_data.get("obstacles", [])
            
            for obstacle in obstacles:
                if obstacle["type"] == "egg" and self.avoid_eggs:
                    self.current_obstacles.append(obstacle)
            
            return True
                
        except Exception as e:
            print("Error updating navigation from camera data: {0}".format(e))
            return False
    
    def _calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        if not pos1 or not pos2:
            return float('inf')
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    
    def _calculate_angle_to_target(self, target_pos):
        """Calculate angle to target from current position"""
        if not self.robot_pos or not target_pos:
            return 0
            
        dx = target_pos[0] - self.robot_pos[0]
        dy = target_pos[1] - self.robot_pos[1]
        return math.degrees(math.atan2(dy, dx))
    
    def _is_obstacle_in_path(self, target_pos):
        """Check if there's an obstacle in the path to the target"""
        if not self.robot_pos or not target_pos or not self.current_obstacles:
            return False
            
        robot_to_target_distance = self._calculate_distance(self.robot_pos, target_pos)
        
        for obstacle in self.current_obstacles:
            obstacle_pos = obstacle["position"]
            
            # Distance from robot to obstacle
            robot_to_obstacle = self._calculate_distance(self.robot_pos, obstacle_pos)
            
            # If obstacle is farther than target, ignore it
            if robot_to_obstacle > robot_to_target_distance:
                continue
                
            # Calculate if obstacle is in the path
            # Vector from robot to target
            rx, ry = self.robot_pos
            tx, ty = target_pos
            target_vector = (tx - rx, ty - ry)
            target_distance = math.sqrt(target_vector[0]**2 + target_vector[1]**2)
            
            # Vector from robot to obstacle
            ox, oy = obstacle_pos
            obstacle_vector = (ox - rx, oy - ry)
            
            # Project obstacle vector onto target vector
            if target_distance > 0:
                dot_product = (obstacle_vector[0] * target_vector[0] + 
                              obstacle_vector[1] * target_vector[1])
                projection = dot_product / target_distance
                
                # If projection is positive and less than distance to target,
                # obstacle might be in the way
                if 0 < projection < target_distance:
                    # Distance from projected point to obstacle
                    proj_x = rx + (projection / target_distance) * target_vector[0]
                    proj_y = ry + (projection / target_distance) * target_vector[1]
                    lateral_distance = self._calculate_distance(
                        (proj_x, proj_y), obstacle_pos)
                    
                    # If obstacle is close to the path
                    if lateral_distance < self.avoid_distance:
                        return True
        
        return False
    
    def _find_path_around_obstacle(self):
        """Calculate an intermediate target to avoid the nearest obstacle"""
        if not self.robot_pos or not self.current_target or not self.current_obstacles:
            return None
            
        # Find closest obstacle
        closest_obstacle = min(self.current_obstacles, 
                              key=lambda o: self._calculate_distance(
                                  self.robot_pos, o["position"]))
        obstacle_pos = closest_obstacle["position"]
        
        # Vector from robot to obstacle
        rx, ry = self.robot_pos
        ox, oy = obstacle_pos
        robot_to_obstacle = (ox - rx, oy - ry)
        
        # Vector from robot to target
        tx, ty = self.current_target["position"]
        robot_to_target = (tx - rx, ty - ry)
        
        # Determine which side to pass the obstacle
        # Cross product to determine if obstacle is on left or right
        cross_product = robot_to_obstacle[0] * robot_to_target[1] - robot_to_obstacle[1] * robot_to_target[0]
        
        # Normalize the vector from robot to obstacle
        distance = math.sqrt(robot_to_obstacle[0]**2 + robot_to_obstacle[1]**2)
        if distance < 1:
            distance = 1
        normalized_x = robot_to_obstacle[0] / distance
        normalized_y = robot_to_obstacle[1] / distance
        
        # Calculate perpendicular vector (rotate 90 degrees)
        if cross_product > 0:
            # Rotate counterclockwise (left)
            perpendicular = (-normalized_y, normalized_x)
        else:
            # Rotate clockwise (right)
            perpendicular = (normalized_y, -normalized_x)
        
        # Calculate intermediate target position
        # Move away from obstacle and perpendicular to the robot-obstacle line
        avoid_distance = self.avoid_distance * 1.5  # Extra margin
        intermediate_x = ox + perpendicular[0] * avoid_distance
        intermediate_y = oy + perpendicular[1] * avoid_distance
        
        return (intermediate_x, intermediate_y)
    
    def navigate(self):
        """
        Main navigation function. Determines the next action based on
        current position, targets, and obstacles.
        
        Returns: Action taken as a string
        """
        if not self.initialized or not self.robot_pos:
            return "ROBOT_NOT_DETECTED"
            
        # If we have no target, stop
        if not self.current_target:
            self.motor_controller.stop()
            return "NO_TARGET"
            
        target_pos = self.current_target["position"]
        distance_to_target = self._calculate_distance(self.robot_pos, target_pos)
            
        # Check if we've reached the target
        if distance_to_target < self.target_precision:
            self.motor_controller.stop()
            
            # Try to pick up the ball
            if self.pickup_controller.initialized:
                self.pickup_controller.pickup()
                
            return "TARGET_REACHED"
            
        # Check if there's an obstacle in the path
        if self._is_obstacle_in_path(target_pos):
            # Find path around obstacle
            intermediate_target = self._find_path_around_obstacle()
            
            if intermediate_target:
                # Move toward intermediate target to avoid obstacle
                return self.motor_controller.move_to_coordinates(
                    self.robot_pos[0], self.robot_pos[1], 
                    self.robot_angle,
                    intermediate_target[0], intermediate_target[1])
            else:
                # No valid path around obstacle, stop
                self.motor_controller.stop()
                return "OBSTACLE_NO_PATH"
        else:
            # No obstacles, move directly to target
            return self.motor_controller.move_to_coordinates(
                self.robot_pos[0], self.robot_pos[1], 
                self.robot_angle,
                target_pos[0], target_pos[1])
    
    def stop(self):
        """Stop all movement"""
        if self.initialized:
            self.motor_controller.stop()
            if self.pickup_controller.initialized:
                self.pickup_controller.stop()

if __name__ == "__main__":
    # Test navigation controller
    nav = NavigationController()
    
    if nav.initialized:
        print("Testing navigation controller...")
        
        # Simulate robot and target positions
        nav.set_robot_position((0, 0), (10, 0))  # Robot at origin, pointing right
        
        # Simulate a target
        test_data = {
            "robot": {
                "head": (10, 0),
                "tail": (0, 0),
                "center": (5, 0),
                "orientation": 0
            },
            "targets": [
                {"type": "white ball", "position": (100, 100), "confidence": 0.9}
            ],
            "obstacles": []
        }
        
        nav.update_from_camera_data(test_data)
        action = nav.navigate()
        print("Navigation action: {0}".format(action))
        
        nav.stop()
        print("Navigation test complete") 