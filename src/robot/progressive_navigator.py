# -*- coding: utf-8 -*-
"""
Progressiv navigator til forbedret boldtargeting med afstandsbaseret præcision
Implementerer 3-fase tilgang: Grov justering -> Fin justering -> Præcisionssigtning
"""

from ..config.settings import *
import math

class ProgressiveNavigator:
    """
    Håndterer intelligent navigation til bolde med progressiv præcision
    """
    
    def __init__(self):
        self.last_angle_diff = None
        self.consecutive_small_moves = 0
        
    def analyze_target_approach(self, distance_cm, angle_diff):
        """
        Analyserer tilgang til mål og returnerer navigation fase
        """
        if distance_cm > PRECISION_DISTANCE:
            return "COARSE"  # Fase 1: Grov justering
        elif distance_cm > DISTANCE_THRESHOLD * 3:
            return "FINE"    # Fase 2: Fin justering
        else:
            return "PRECISION"  # Fase 3: Præcisionssigtning
    
    def get_phase_parameters(self, phase):
        """
        Returnerer parametre for en given navigationsfase
        """
        if phase == "COARSE":
            return {
                "turn_threshold": COARSE_TURN_THRESHOLD,
                "max_turn": MAX_TURN_COARSE, 
                "max_forward": MAX_FORWARD_DISTANCE_FAR,
                "turn_speed": ROBOT_TURN_SPEED,
                "forward_speed": ROBOT_FORWARD_SPEED
            }
        elif phase == "FINE":
            return {
                "turn_threshold": FINE_TURN_THRESHOLD,
                "max_turn": MAX_TURN_FINE,
                "max_forward": MAX_FORWARD_DISTANCE_NEAR, 
                "turn_speed": ROBOT_TURN_SPEED,
                "forward_speed": ROBOT_FORWARD_SPEED
            }
        else:  # PRECISION
            return {
                "turn_threshold": VERY_FINE_TURN_THRESHOLD,
                "max_turn": MAX_TURN_PRECISION,
                "max_forward": MAX_FORWARD_DISTANCE_CLOSE,
                "turn_speed": ROBOT_TURN_SPEED_SLOW,  # Langsom til præcis sigtning
                "forward_speed": ROBOT_FORWARD_SPEED_SLOW
            }
    
    def should_turn(self, angle_diff, phase_params):
        """
        Beslutter om robotten skal dreje baseret på vinkel og fase
        """
        return abs(angle_diff) > phase_params["turn_threshold"]
    
    def calculate_turn_command(self, angle_diff, phase_params):
        """
        Beregner optimal drejning kommando MED ROTATION-BASERET PRÆCISION
        """
        direction = "right" if angle_diff > 0 else "left"
        turn_amount = min(abs(angle_diff), phase_params["max_turn"])
        
        # BRUG ROTATION I STEDET FOR TID - meget mere præcist!
        return {
            "command": "simple_turn",
            "direction": direction, 
            "angle_degrees": turn_amount,  # NØGLE: Send vinkel for rotation-baseret bevægelse
            "speed": phase_params["turn_speed"],
            "angle_amount": turn_amount
        }
    
    def calculate_forward_command(self, distance_cm, angle_accuracy, phase_params):
        """
        Beregner optimal fremad kommando
        """
        # Begræns afstand baseret på fase og vinkel nøjagtighed
        max_distance = phase_params["max_forward"]
        
        # Hvis retningen ikke er præcis nok, kør kun kort distance
        if abs(angle_accuracy) > VERY_FINE_TURN_THRESHOLD:
            max_distance = min(max_distance, MAX_FORWARD_DISTANCE_CLOSE)
            
        move_distance = min(distance_cm, max_distance)
        
        return {
            "command": "simple_forward",
            "distance": move_distance,
            "speed": phase_params["forward_speed"]
        }
    
    def get_navigation_command(self, angle_diff, distance_cm):
        """
        Hovedfunktion: returnerer optimal navigation kommando
        """
        # Analyser hvilken fase vi er i
        phase = self.analyze_target_approach(distance_cm, angle_diff)
        phase_params = self.get_phase_parameters(phase)
        
        navigation_info = {
            "phase": phase,
            "should_turn": self.should_turn(angle_diff, phase_params),
            "angle_diff": angle_diff,
            "distance_cm": distance_cm
        }
        
        # Bestem kommando type
        if navigation_info["should_turn"]:
            command = self.calculate_turn_command(angle_diff, phase_params)
            command["navigation_phase"] = phase
            self.last_angle_diff = angle_diff
            return command, navigation_info
            
        elif distance_cm > DISTANCE_THRESHOLD:
            command = self.calculate_forward_command(distance_cm, angle_diff, phase_params) 
            command["navigation_phase"] = phase
            return command, navigation_info
            
        else:
            # Meget tæt på - final push
            command = {
                "command": "simple_forward",
                "distance": 3,
                "speed": ROBOT_FORWARD_SPEED_SLOW,
                "navigation_phase": "FINAL"
            }
            return command, navigation_info
    
    def get_status_message(self, phase, command, navigation_info):
        """
        Returnerer beskrivende status besked
        """
        if command["command"] == "simple_turn":
            speed_desc = "SLOW" if command.get("speed") == ROBOT_TURN_SPEED_SLOW else "NORMAL"
            return "PHASE {}: TURNING {} {:.1f}° ({} speed)".format(
                phase, command["direction"], command.get("angle_amount", 0), speed_desc)
        elif command["command"] == "simple_forward":
            speed_desc = "SLOW" if command.get("speed") == ROBOT_FORWARD_SPEED_SLOW else "NORMAL" 
            return "PHASE {}: FORWARD {:.1f}cm ({} speed)".format(
                phase, command["distance"], speed_desc)
        else:
            return "FINAL PUSH: {:.1f}cm".format(command["distance"]) 