# -*- coding: utf-8 -*-
"""
Robot controller der håndterer motorer og grundlæggende operationer (ingen sensorer)
"""

from ev3dev2.motor import LargeMotor, MoveTank, MediumMotor
from ev3dev2.button import Button
from time import sleep
from ..config.settings import *

class RobotController:
    
    def __init__(self):
        print("Initializing robot controller...")
        
        # Initialiserer motorer (port A og D)
        self.left_motor = LargeMotor('outA')
        self.right_motor = LargeMotor('outD')
        self.tank_drive = MoveTank('outA', 'outD')

        # Medium motor for collect mechanism
        self.collect_motor = MediumMotor('outC')
        
        try:
            self.collect_motor.on(speed=-50)  # Start immediately
            print("Collect mechanism started (Port C)")
        except Exception as e:
            print("Failed to start collect mechanism:", e)

        # Initialiserer button (hvis tilgængelig)
        try:
            self.button = Button()
        except:
            self.button = None
            print("No button available")
        
        # State
        self.running = True
        
        # Start harvester motor konstant
        self.start_harvester()
        
        print("Robot controller initialized")
    
    def start_harvester(self):
        """Starter harvester motoren konstant baglæns"""
        if self.harvester_motor:
            try:
                # Kør konstant baglæns på højere speed for hurtigere opsamling
                self.harvester_motor.on(-50)  # Øget fra -30 til -50 for hurtigere samling
                print("Harvester motor started - running backwards constantly at speed 50")
            except Exception as e:
                print("Error starting harvester: {}".format(e))
    
    def stop_all_motors(self):
        """Emergency stop for alle motorer"""
        try:
            self.tank_drive.off()
            self.left_motor.stop()
            self.right_motor.stop()
            self.collect_motor.off()
         
            print("All motors stopped")
        except Exception as e:
            print("Error stopping motors: {}".format(e))
    
    def check_buttons(self):
        """Tjekker for BACK knap tryk for at stoppe robotten"""
        if self.button and self.button.backspace:
            print("Back button pressed - stopping robot")
            self.running = False
            self.stop_all_motors()
            return True
        return False
    
    def normalize_angle(self, angle):
        """Normaliserer vinkel til [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle 
    
    def get_current_heading(self):
        """Returnerer nuværende retning - ingen gyro, så returnerer 0"""
        return 0  # Ingen gyro - kun vision navigation
    
    def simple_turn(self, direction, angle_degrees):
        """
        Drej roboten med en given vinkel - harvester fortsætter med at køre
        """
        speed = ROBOT_TURN_SPEED
        
        # Minimal vinkel tærskel - EV3 motorer kan ikke dreje under ~5 grader pålideligt
        min_angle = 5.0
        actual_angle = max(abs(angle_degrees), min_angle)
        
        # Beregn omdrejninger baseret på vinkel
        # 0.5 omdrejninger = 90 grader, så rotations = (angle / 90) * 0.5
        rotations = actual_angle / 90.0 * 0.5
        
        print("Simple turn: {} {:.1f} degrees -> {:.1f} degrees ({:.3f} rotations)".format(
            direction, angle_degrees, actual_angle, rotations))
        
        # Spring over hvis vinkel er for lille
        if rotations < 0.05:  # Under 0.05 omdrejninger = ingen bevægelse
            print("Angle too small, skipping turn")
            return
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward
            self.tank_drive.on_for_rotations(speed, -speed, rotations)
        else:
            # Turn left: left motor backward, right motor forward
            self.tank_drive.on_for_rotations(-speed, speed, rotations)

        print("Simple turn complete - harvester still running")
    
    def turn_by_angle(self, angle_degrees):
        """
            Turn robot by specific angle using motor rotations
            Positive angle = right turn, negative angle = left turn
        """
        if angle_degrees == 0:
            return
            
        direction = "right" if angle_degrees > 0 else "left"
        self.simple_turn(direction, abs(angle_degrees))

    def simple_forward(self, distance_cm, overshoot_cm=10):
        """
        Move robot forward - harvester fortsætter med at køre
        """
        speed = ROBOT_FORWARD_SPEED
        
        # Beregn total afstand inklusiv overshoot
        total_distance = distance_cm + overshoot_cm
        
        # Beregn omdrejninger baseret på hjul diameter
        wheel_diameter_cm = 68.8  # Korrekt hjul diameter fra specs
        wheel_circumference_cm = 3.14159 * wheel_diameter_cm  # π × diameter ≈ 216 cm
        rotations = total_distance / wheel_circumference_cm
        
        print("Simple forward: {:.1f} cm + {:.1f} cm overshoot = {:.1f} cm total ({:.3f} rotations)".format(
            distance_cm, overshoot_cm, total_distance, rotations))
        
        # Begge motorer fremad (motorer er omvendt, så bruger negative værdier)
        self.tank_drive.on_for_rotations(-speed, -speed, rotations)
        
        print("Simple forward complete")

 
    def cleanup(self):
        """Afslutter robotten og stopper alle motorer"""
        print("Cleaning up robot controller...")
        self.stop_all_motors()
        self.running = False 

        def release_harvester(self):
        "aflevere bolde"
        if self.harvester_motor:
            try:
                # Kør konstant baglæns på højere speed for hurtigere opsamling
                self.harvester_motor.on(30)  # Øget fra -30 til -50 for hurtigere samling
                print("Harvester motor started - running backwards constantly at speed 50")
            except Exception as e:
                print("Error starting harvester: {}".format(e))

    