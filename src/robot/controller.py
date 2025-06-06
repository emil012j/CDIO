# -*- coding: utf-8 -*-
"""
 Initialiserer motorer/gyro, kalibrerer gyro, emergency stop, tjekker knap tryk
"""

from ev3dev2.motor import LargeMotor, MoveTank
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.button import Button
import math
from time import sleep
import socket
import json
import threading
from ..config.settings import *

#robot controller, der håndterer motorer, sensorer, og basic operations
class RobotController:
    
    def __init__(self):
        print("Initializing robot controller...")
        
        # Initialiserer motorer (port A og D)
        self.left_motor = LargeMotor('outA')
        self.right_motor = LargeMotor('outD')
        self.tank_drive = MoveTank('outA', 'outD')
        
        # Initialiserer gyro sensor (port 1)
        self.gyro = GyroSensor('in1')
        self.button = Button()
        
        # State
        self.running = True
        
        print("Robot controller initialized")
    
    #kalibrerer gyroen
    def calibrate_gyro(self):
        print("Calibrating gyro...")
        self.gyro.reset()
        sleep(1)
        self.gyro.calibrate()  # Kalibrerer gyro ved start
        sleep(1)
        print("Gyro calibrated. Start angle: {}".format(self.gyro.angle))
    
    #emergency stop for alle motorer som bruges ved at trykke escape på pc'en virker ikke ordentligt
    def stop_all_motors(self):
        try:
            self.tank_drive.off()  # Håndterer emergency stop
            self.left_motor.stop()
            self.right_motor.stop()
            print("All motors stopped")
        except Exception as e:
            print("Error stopping motors: {}".format(e))
    
    #tjekker for knap tryk, virker ikke ordentligt
    def check_buttons(self):
        if self.button.backspace:  # Tjekker for knap tryk (BACK button)
            print("Back button pressed - stopping robot")
            self.running = False
            self.stop_all_motors()
            return True
        return False
    
    #normaliserer vinklen til [-180, 180] range for at undgå at vinklen bliver for stor, da gyroen kan give vinkler over 360 grader
    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    #henter den nuværende heading fra gyroen
    def get_current_heading(self):
        try:
            return self.gyro.angle
        except Exception:
            return 0
    
    #afslutter robotten
    def cleanup(self):
        print("Cleaning up robot controller...")
        self.stop_all_motors()
        self.running = False 