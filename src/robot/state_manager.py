# -*- coding: utf-8 -*-
"""
Robot State Manager - håndterer robot tilstand og bold targeting logik
"""

from time import time
import math
from ..config.settings import MEMORY_TIMEOUT_SECONDS

class RobotStateManager:
    """
    Håndterer robot tilstand, bold prioritering og capture sekvens
    Inkluderer target memory for at håndtere når bolde bliver skygget
    """
    
    def __init__(self):
        self.current_target_ball = None
        self.target_memory = {}  # Gemmer sidst kendte positioner for bolde
        self.last_capture_time = 0
        self.capture_cooldown = 2.0  # sekunder mellem captures
        self.balls_captured = 0
        self.lost_ball_timeout = MEMORY_TIMEOUT_SECONDS  # sekunder før en lost ball opgives
        self.memory_update_distance = 10.0  # pixels - min afstand før position opdateres
        
    def update_ball_memory(self, all_balls):
        """
        Opdaterer target memory med nuværende synlige bolde
        """
        current_time = time()
        
        # Opdater memory for alle synlige bolde
        for ball in all_balls:
            ball_id = self._get_ball_id(ball)
            ball_pos = ball["pos"] if isinstance(ball, dict) else ball
            
            # Opdater kun hvis position har ændret sig signifikant
            if (ball_id not in self.target_memory or 
                self._position_changed_significantly(ball_pos, self.target_memory[ball_id]["pos"])):
                
                self.target_memory[ball_id] = {
                    "pos": ball_pos,
                    "last_seen": current_time,
                    "confidence": ball.get("confidence", 1.0) if isinstance(ball, dict) else 1.0
                }
        
        # Marker bolde som lost hvis de ikke er set i et stykke tid
        for ball_id in list(self.target_memory.keys()):
            if current_time - self.target_memory[ball_id]["last_seen"] > self.lost_ball_timeout:
                print(f"Ball {ball_id} marked as lost (timeout)")
                del self.target_memory[ball_id]
    
    def _get_ball_id(self, ball):
        """
        Genererer et unikt ID for en bold baseret på position
        """
        pos = ball["pos"] if isinstance(ball, dict) else ball
        # Runde til nærmeste 10 pixels for at undgå støj
        return f"{int(pos[0]/10)*10}_{int(pos[1]/10)*10}"
    
    def _position_changed_significantly(self, pos1, pos2):
        """
        Tjekker om position har ændret sig signifikant
        """
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return distance > self.memory_update_distance
    
    def select_target_ball(self, robot_head, all_balls):
        """
        Vælger den bedste target bold baseret på afstand fra robot head
        Bruger både synlige bolde og target memory
        """
        if not robot_head:
            return None
        
        # Opdater memory først
        if all_balls:
            self.update_ball_memory(all_balls)
        
        # Hvis vi allerede har en target, tjek om den stadig er valid
        if self.current_target_ball:
            target_id = self._get_ball_id(self.current_target_ball)
            if target_id in self.target_memory:
                # Target er stadig valid, brug memory position
                memory_pos = self.target_memory[target_id]["pos"]
                print(f"Using target memory for ball at {memory_pos}")
                return {
                    "pos": memory_pos,
                    "from_memory": True,
                    "ball_id": target_id
                }
        
        # Find nye targets fra alle tilgængelige bolde (synlige + memory)
        best_ball = None
        best_distance = float('inf')
        
        # Tjek synlige bolde først (prioritet)
        if all_balls:
            for ball in all_balls:
                ball_pos = ball["pos"] if isinstance(ball, dict) else ball
                distance = self._calculate_distance(robot_head["pos"], ball_pos)
                
                if distance < best_distance:
                    best_distance = distance
                    best_ball = ball
                    
        # Hvis ingen synlige bolde, brug memory
        if not best_ball and self.target_memory:
            for ball_id, ball_data in self.target_memory.items():
                distance = self._calculate_distance(robot_head["pos"], ball_data["pos"])
                
                if distance < best_distance:
                    best_distance = distance
                    best_ball = {
                        "pos": ball_data["pos"],
                        "from_memory": True,
                        "ball_id": ball_id
                    }
        
        self.current_target_ball = best_ball
        return best_ball
    
    def _calculate_distance(self, pos1, pos2):
        """
        Beregner afstand mellem to positioner
        """
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return (dx * dx + dy * dy) ** 0.5
    
    def is_ball_lost(self, ball):
        """
        Tjekker om en bold er lost (ikke synlig men stadig i memory)
        """
        if isinstance(ball, dict) and ball.get("from_memory", False):
            return True
        return False
    
    def mark_ball_captured(self, ball):
        """
        Markerer at en bold er blevet captured og fjerner den fra memory
        """
        self.last_capture_time = time()
        self.balls_captured += 1
        
        # Fjern captured ball fra memory
        if ball:
            ball_id = ball.get("ball_id") or self._get_ball_id(ball)
            if ball_id in self.target_memory:
                del self.target_memory[ball_id]
                print(f"Removed captured ball {ball_id} from memory")
        
        # Reset current target
        self.current_target_ball = None
        print(f"Ball captured! Total: {self.balls_captured}")
    
    def is_ready_for_capture(self):
        """
        Tjekker om robotten er klar til en ny capture (cooldown)
        """
        current_time = time()
        return (current_time - self.last_capture_time) >= self.capture_cooldown
    
    def get_remaining_balls(self, all_balls=None):
        """
        Returnerer antal resterende bolde (synlige + memory)
        """
        visible_count = len(all_balls) if all_balls else 0
        memory_count = len(self.target_memory)
        # Undgå dobbelt-optælling af synlige bolde der også er i memory
        total = max(visible_count, memory_count)
        return total
    
    def reset_target(self):
        """
        Nulstiller current target (fx efter capture)
        """
        self.current_target_ball = None
    
    def get_memory_status(self):
        """
        Returnerer status for target memory (til debugging)
        """
        current_time = time()
        status = {}
        for ball_id, ball_data in self.target_memory.items():
            age = current_time - ball_data["last_seen"]
            status[ball_id] = {
                "pos": ball_data["pos"],
                "age_seconds": age,
                "confidence": ball_data["confidence"]
            }
        return status 