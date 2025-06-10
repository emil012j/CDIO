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
        
        # Initialiserer motorer (port A og D for hjul, port C for harvester)
        self.left_motor = LargeMotor('outA')
        self.right_motor = LargeMotor('outD')
        self.tank_drive = MoveTank('outA', 'outD')
        
        # Harvester motor (Motor C) - skal køre baglæns hele tiden
        try:
            self.harvester_motor = MediumMotor('outC')
            self.harvester_running = False
            print("Harvester motor (Motor C) initialized")
        except:
            self.harvester_motor = None
            print("Warning: Could not initialize harvester motor (Motor C)")

        # Initialiserer button (hvis tilgængelig)
        try:
            self.button = Button()
        except:
            self.button = None
            print("No button available")
        
        # State
        self.running = True
        
        # Start harvester motor
        self.start_harvester()
        
        print("Robot controller initialized")
    
    def stop_all_motors(self):
        """Emergency stop for alle motorer inklusiv harvester"""
        try:
            self.tank_drive.off()
            self.left_motor.stop()
            self.right_motor.stop()
            
            # Stop også harvester motor
            if self.harvester_motor:
                self.harvester_motor.off()
                self.harvester_running = False
         
            print("All motors stopped (including harvester)")
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
    
    def start_harvester(self):
        """Starter harvester motoren (kører baglæns konstant på speed 45)"""
        if self.harvester_motor:
            try:
                self.harvester_motor.on(POWER_HARVESTER)  # Speed 45 konstant
                self.harvester_running = True
                print("Harvester motor started at speed {} (running constantly)".format(POWER_HARVESTER))
            except Exception as e:
                print("Error starting harvester: {}".format(e))
        else:
            print("Warning: Harvester motor not available")

    def stop_harvester(self):
        """Stopper harvester motoren"""
        if self.harvester_motor:
            try:
                self.harvester_motor.off()
                self.harvester_running = False
                print("Harvester motor stopped")
            except Exception as e:
                print("Error stopping harvester: {}".format(e))
        else:
            print("Warning: Harvester motor not available")
    
    def ensure_harvester_running(self):
        """Sikrer at harvester kører konstant på speed 45"""
        if self.harvester_motor:
            try:
                if not self.harvester_running:
                    self.harvester_motor.on(POWER_HARVESTER)
                    self.harvester_running = True
                    print("Restarted harvester at speed {}".format(POWER_HARVESTER))
                # Double check - restart hvis den ikke kører
                elif self.harvester_motor.speed == 0:
                    self.harvester_motor.on(POWER_HARVESTER)
                    print("Harvester was stopped - restarted at speed {}".format(POWER_HARVESTER))
            except Exception as e:
                print("Error ensuring harvester: {}".format(e))

    def simple_turn(self, direction, duration=None, speed=None, angle_degrees=None):
        """ROTATION-BASERET drejning - meget mere præcist end tid!"""
        if speed is None:
            speed = ROBOT_TURN_SPEED
            
        # Sikr at harvester kører under drejning
        self.ensure_harvester_running()
        
        # Hvis vinkel er angivet, beregn rotations - MEGET MERE PRÆCIST!
        if angle_degrees is not None:
            # 90° = 0.5 rotation, så X° = (X/90) * 0.5 rotations
            rotations = (abs(angle_degrees) / 90.0) * MOTOR_ROTATIONS_PER_90_DEGREES
            
            print("ROTATION-BASED turn: {} {:.1f}° = {:.3f} rotations at speed {} (harvester speed {})".format(
                direction, abs(angle_degrees), rotations, speed, POWER_HARVESTER))
            
            if direction == "right":
                # Right turn: left forward, right backward
                self.left_motor.on_for_rotations(speed, rotations, brake=True, block=False)
                self.right_motor.on_for_rotations(-speed, rotations, brake=True, block=True)
            else:
                # Left turn: left backward, right forward  
                self.left_motor.on_for_rotations(-speed, rotations, brake=True, block=False)
                self.right_motor.on_for_rotations(speed, rotations, brake=True, block=True)
                
        else:
            # Fallback til tidsbaseret hvis ingen vinkel angivet
            if duration is None:
                duration = 1.0  # Default duration
            print("TIME-based turn: {} for {:.2f} seconds at speed {} (harvester speed {})".format(
                direction, duration, speed, POWER_HARVESTER))
            
            if direction == "right":
                self.tank_drive.on(speed, -speed)
            else:
                self.tank_drive.on(-speed, speed)
                
            sleep(duration)
            self.tank_drive.off()
        
        # Sikr harvester stadig kører efter drejning
        self.ensure_harvester_running()
        print("Turn complete - harvester still running")
    
    def precision_turn(self, direction, angle_degrees):
        """Præcisionsdrejning baseret på motor rotations - MEGET PRÆCIST!"""
        # Sikr at harvester kører under drejning
        self.ensure_harvester_running()
        
        # Vælg hastighed baseret på vinkelstørrelse
        if abs(angle_degrees) > COARSE_TURN_THRESHOLD:
            speed = ROBOT_TURN_SPEED
        elif abs(angle_degrees) > FINE_TURN_THRESHOLD:
            speed = ROBOT_TURN_SPEED_SLOW
        else:
            speed = ROBOT_TURN_SPEED_SLOW
        
        # Beregn præcise motor rotations
        rotations = (abs(angle_degrees) / 90.0) * MOTOR_ROTATIONS_PER_90_DEGREES
        
        print("PRECISION turn: {} {:.1f}° = {:.4f} rotations at speed {} (harvester speed {})".format(
            direction, abs(angle_degrees), rotations, speed, POWER_HARVESTER))
            
        if direction == "right":
            self.left_motor.on_for_rotations(speed, rotations, brake=True, block=False)
            self.right_motor.on_for_rotations(-speed, rotations, brake=True, block=True)
        else:
            self.left_motor.on_for_rotations(-speed, rotations, brake=True, block=False)
            self.right_motor.on_for_rotations(speed, rotations, brake=True, block=True)
        
        # Sikr harvester stadig kører efter drejning
        self.ensure_harvester_running()
        print("Precision turn complete - harvester still running")

    def simple_forward(self, distance_cm, speed=None):
        """ROTATION-BASERET fremadkørsel - baseret på hjulomkreds"""
        if speed is None:
            speed = ROBOT_FORWARD_SPEED
            
        # Sikr at harvester kører under kørsel (vigtigt for at samle bolde!)
        self.ensure_harvester_running()
            
        # Beregn motor rotations baseret på hjulomkreds
        # distance_cm / omkreds_cm = antal rotationer
        rotations = distance_cm / WHEEL_CIRCUMFERENCE_CM
        
        print("ROTATION-BASED forward: {:.1f} cm = {:.3f} rotations at speed {} (harvester collecting at speed {})".format(
            distance_cm, rotations, speed, POWER_HARVESTER))
        
        # Begge motorer fremad (negative speed for fremad)
        self.left_motor.on_for_rotations(-speed, rotations, brake=True, block=False)
        self.right_motor.on_for_rotations(-speed, rotations, brake=True, block=True)
        
        # Sikr harvester stadig kører efter kørsel
        self.ensure_harvester_running()
        print("Forward complete - harvester still collecting")
    
    def precision_forward(self, distance_cm):
        """Præcisionskørsel med variabel hastighed baseret på afstand"""
        # Sikr at harvester kører under kørsel
        self.ensure_harvester_running()
        
        if distance_cm > PRECISION_DISTANCE:
            speed = ROBOT_FORWARD_SPEED
            move_distance = min(distance_cm, MAX_FORWARD_DISTANCE_FAR)
        elif distance_cm > DISTANCE_THRESHOLD * 2:
            speed = ROBOT_FORWARD_SPEED_SLOW
            move_distance = min(distance_cm, MAX_FORWARD_DISTANCE_NEAR)
        else:
            speed = ROBOT_FORWARD_SPEED_SLOW
            move_distance = min(distance_cm, MAX_FORWARD_DISTANCE_CLOSE)
            
        self.simple_forward(move_distance, speed)
        return move_distance

    def cleanup(self):
        """Afslutter robotten og stopper alle motorer"""
        print("Cleaning up robot controller...")
        self.stop_all_motors()  # Dette stopper også harvesteren
        self.running = False 