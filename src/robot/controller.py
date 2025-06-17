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
        
        # Re-enabled: Harvester motor needed for ball collection
        try:
            self.collect_motor.on(speed=-100)  # HURTIGERE: Øget collect speed fra -30 til -50
            print("Collect mechanism started (Port C) - speed -50")
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
        
        print("Robot controller initialized")
    
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
        Drej roboten med en given vinkel
        Minimal vinkel: 5 grader
        """
        speed = ROBOT_TURN_SPEED
        
        # Minimal vinkel tærskel - EV3 motorer kan ikke dreje under ~5 grader pålideligt
        min_angle = 2.0
        actual_angle = max(abs(angle_degrees), min_angle)
        
        # Beregn omdrejninger baseret på vinkel
        # 0.5 omdrejninger = 90 grader, så rotations = (angle / 90) * 0.5
        rotations = actual_angle / 90.0 * 0.5
        
        print("Simple turn: {} {:.1f} degrees -> {:.1f} degrees ({:.3f} rotations)".format(
            direction, angle_degrees, actual_angle, rotations))
        
        # Spring over hvis vinkel er for lille
        if rotations < 0.01:  # Under 0.05 omdrejninger = ingen bevægelse
            print("Angle too small, skipping turn")
            return
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motorer er omvendt)
            self.tank_drive.on_for_rotations(speed, -speed, rotations)
        else:
            # Turn left: left motor backward, right motor forward (motorer er omvendt)  
            self.tank_drive.on_for_rotations(-speed, speed, rotations)
            
        print("Simple turn complete")
    
    def simple_turn_rotation(self, direction, rotations):
        """
        Drej roboten med direkte omdrejninger (INGEN angle konvertering)
        rotations: antal omdrejninger (0.5 = 90°, 1.0 = 180°)
        """
        speed = ROBOT_TURN_SPEED
        
        print("Simple turn rotation: {} {:.3f} rotations (DIRECT - no angle conversion)".format(
            direction, rotations))
        
        # Spring over hvis rotation er for lille - reduceret for bedre præcision
        if rotations < 0.0000001:  # Reduceret fra 0.01 til 0.003 for fine-tuning
            print("Rotation too small, skipping turn")
            return
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motorer er omvendt)
            self.tank_drive.on_for_rotations(speed, -speed, rotations)
        else:
            # Turn left: left motor backward, right motor forward (motorer er omvendt)  
            self.tank_drive.on_for_rotations(-speed, speed, rotations)
            
        print("Simple turn rotation complete")
    
    def turn_by_angle(self, angle_degrees):
        """
            Turn robot by specific angle using motor rotations
            Positive angle = right turn, negative angle = left turn
        """
        if angle_degrees == 0:
            return
            
        direction = "right" if angle_degrees > 0 else "left"
        self.simple_turn(direction, abs(angle_degrees))

    def simple_forward(self, distance_cm, overshoot_cm=0):
        """
        Move robot forward using motor rotations - NO MORE OVERSHOOT BY DEFAULT
        Hjul diameter: 68.8 mm = 6.88 cm → omkreds: ~21.6 cm per omdrejning
        """
        speed = ROBOT_FORWARD_SPEED
        
        # Beregn total afstand inklusiv overshoot (nu 0 som standard)
        total_distance = distance_cm + overshoot_cm
        
        # Beregn omdrejninger baseret på hjul diameter
        wheel_diameter_cm = 6.88  # 68.8 mm = 6.88 cm korrekt måling af hjul diameter  
        wheel_circumference_cm = 3.14159 * wheel_diameter_cm  # π × diameter ≈ 21.6 cm
        rotations = total_distance / wheel_circumference_cm
        
        if overshoot_cm > 0:
            print("Simple forward: {:.1f} cm + {:.1f} cm overshoot = {:.1f} cm total ({:.3f} rotations)".format(
                distance_cm, overshoot_cm, total_distance, rotations))
        else:
            print("Simple forward: {:.1f} cm ({:.3f} rotations) - NO OVERSHOOT".format(
                distance_cm, rotations))
        
        # Begge motorer fremad (motorer er omvendt, så bruger negative værdier)
        self.tank_drive.on_for_rotations(-speed, -speed, rotations)
        
        print("Simple forward complete")
    


    def simple_backward(self, distance_cm):
        """
        Move robot backward using motor rotations
        """
        speed = ROBOT_FORWARD_SPEED
        
        # Beregn omdrejninger baseret på hjul diameter  
        wheel_diameter_cm = 6.88  # 68.8 mm = 6.88 cm korrekt måling af hjul diameter
        wheel_circumference_cm = 3.14159 * wheel_diameter_cm  # π × diameter ≈ 21.6 cm
        rotations = distance_cm / wheel_circumference_cm
        
        print("Simple backward: {:.1f} cm ({:.3f} rotations)".format(distance_cm, rotations))
        
        # Begge motorer bagud (motorer er omvendt, så bruger positive værdier)
        self.tank_drive.on_for_rotations(speed, speed, rotations)
        
        print("Simple backward complete")

    def turn_180_degrees(self):
        """
        Præcis 180 graders drejning baseret på motor rotations
        180 grader = 1 omgang med hjulene i hver sin retning = 1.0 rotation
        """
        print("Turning 180 degrees using direct rotation method")
        self.simple_turn_rotation("right", 1.0)  # 1.0 rotation = 180°
        print("180 degree turn complete")
    
    def blind_ball_collection(self):
        """
        BLIND BALL COLLECTION SEQUENCE:
        1. Kør 20 cm lige ud (ca 1 omdrejning)
        2. Bak 20 cm bagud (ca 1 omdrejning) 
        3. Roter 180 grader (ca 1 omdrejning i hver retning)
        """
        print("*** BLIND BALL COLLECTION ***")
        
        # Step 1: Kør 20 cm frem (blind - ligemeget om bold forsvinder)
        print("STEP 1: Driving forward 20 cm (BLIND)")
        self.simple_forward(20, overshoot_cm=0)  # Ingen overshoot - præcis 20 cm
        
        # Step 2: Bak 20 cm bagud
        print("STEP 2: Backing up 20 cm")
        self.simple_backward(20)
        
        # Step 3: Roter 180 grader for næste bold
        print("STEP 3: Rotating 180 degrees for next ball")
        # self.turn_180_degrees()
        
        print("*** BLIND BALL COLLECTION COMPLETE ***")


    def release_balls(self, duration=4):
        """
        Kør collect_motor i modsat retning for at frigive bolde.
        duration: hvor længe motoren skal køre (sekunder)
        """
        print("Releasing balls")
        try:
            self.collect_motor.on(speed=100)  # Kør fremad (modsat af opsamling)
            sleep(duration)
            self.collect_motor.on(-100)
            print("Balls released")
        except Exception as e:
            print("Error releasing balls:", e)

    def cleanup(self):
        """Afslutter robotten og stopper alle motorer"""
        print("Cleaning up robot controller...")
        self.stop_all_motors()
        self.running = False 