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
    
    def simple_turn(self, direction, duration):
        """Simple turn uden gyro - kun tidsbaseret"""
        speed = ROBOT_TURN_SPEED
        print("Simple turn: {} for {:.2f} seconds".format(direction, duration))
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motorer er omvendt)
            self.tank_drive.on(speed, -speed)
       
        else:
            # Turn left: left motor backward, right motor forward (motorer er omvendt)  
            self.tank_drive.on(-speed, speed)
           
            
        sleep(duration)
        self.tank_drive.off()
        print("Simple turn complete")

    def simple_forward(self, distance_cm):
        """Simple forward uden gyro - kun tidsbaseret"""
        speed = ROBOT_FORWARD_SPEED
        # Estimerer tid baseret på afstand
        duration = distance_cm / ESTIMATED_FORWARD_RATE
        
        print("Simple forward: {:.1f} cm for {:.2f} seconds".format(distance_cm, duration))
        
        # Begge motorer fremad (motorer er omvendt, så bruger negative værdier)
        self.tank_drive.on(-speed, -speed)

        sleep(duration)
        self.tank_drive.off()
        print("Simple forward complete")

    def cleanup(self):
        """Afslutter robotten og stopper alle motorer"""
        print("Cleaning up robot controller...")
        self.stop_all_motors()
        self.running = False 