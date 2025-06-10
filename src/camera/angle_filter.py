# -*- coding: utf-8 -*-
"""
Angle Filter - glatter vinkel målinger for at undgå oscillation
"""

class AngleFilter:
    """
    Simpel filter til at glatte angle measurements og forhindre oscillation
    """
    
    def __init__(self, history_size=3, change_threshold=30.0):
        self.history = []
        self.history_size = history_size
        self.change_threshold = change_threshold  # grader - øget fra 15 til 30 for mindre aggressiv filtrering
        self.last_filtered_angle = None
        
    def filter_angle(self, new_angle):
        """
        Filtrer ny vinkel måling
        Returns: filtered angle
        """
        # Tilføj til history
        self.history.append(new_angle)
        if len(self.history) > self.history_size:
            self.history.pop(0)
        
        # Hvis vi ikke har nok history, returner som den er
        if len(self.history) < 2:
            self.last_filtered_angle = new_angle
            return new_angle
        
        # Beregn gennemsnit af recent målinger
        avg_angle = sum(self.history) / len(self.history)
        
        # Hvis ændringen er for stor, begræns den
        if self.last_filtered_angle is not None:
            angle_change = abs(avg_angle - self.last_filtered_angle)
            if angle_change > self.change_threshold:
                # Begræns ændringen
                if avg_angle > self.last_filtered_angle:
                    filtered_angle = self.last_filtered_angle + self.change_threshold
                else:
                    filtered_angle = self.last_filtered_angle - self.change_threshold
            else:
                filtered_angle = avg_angle
        else:
            filtered_angle = avg_angle
        
        self.last_filtered_angle = filtered_angle
        return filtered_angle
    
    def reset(self):
        """
        Nulstiller filter
        """
        self.history = []
        self.last_filtered_angle = None 