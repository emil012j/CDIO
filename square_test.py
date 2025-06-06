#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1
from ev3dev2.button import Button
from time import sleep

class SquareTest:
    def __init__(self):
        # Motors - A (left) and D (right), motors are reversed (minus = forward)
        self.tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        
        # Gyro sensor
        self.gyro = GyroSensor(INPUT_1)
        self.btn = Button()
        
        # PID constants for straight driving - optimized for 7cm wheels
        self.KP = 0.5  # Reduced for less aggressive correction
        self.KI = 0.01  # Reduced to avoid oversteering
        self.KD = 0.3  # Increased for better damping
        self.integral = 0
        self.last_error = 0
        
        # Calibrate gyro at start
        print("Calibrating gyro...")
        self.reset_gyro()
        print("Gyro calibrated. Start angle: {}".format(self.gyro.angle))
    
    def reset_gyro(self):
        """Reset and calibrate gyro sensor"""
        self.gyro.mode = "GYRO-CAL"
        sleep(0.2)
        self.gyro.mode = "GYRO-ANG"
        sleep(0.2)
        self.integral = 0
        self.last_error = 0
    
    def normalize_angle(self, angle):
        """Normalize angle to range [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def turn_to_angle(self, target_angle, speed=15):
        """Turn to specific angle with gyro precision"""
        current_angle = self.gyro.angle
        angle_to_turn = self.normalize_angle(target_angle - current_angle)
        
        print("Turning from {} degrees to {} degrees (need to turn {} degrees)".format(
            current_angle, target_angle, angle_to_turn))
        
        # Precise turning with decreasing speed
        while abs(self.gyro.angle - target_angle) > 1:  # 1 degree tolerance
            remaining = abs(target_angle - self.gyro.angle)
            current_speed = min(speed, max(5, remaining * 0.4))
            
            if angle_to_turn > 0:  # Turn right
                self.tank_drive.on(current_speed, -current_speed)  # Motors are reversed
            else:  # Turn left
                self.tank_drive.on(-current_speed, current_speed)  # Motors are reversed
            
            print("Angle: {} degrees, Remaining: {} degrees".format(
                self.gyro.angle, remaining))
                
        self.tank_drive.off()
        print("Turn complete. Final angle: {} degrees".format(self.gyro.angle))
    
    def drive_straight(self, distance_cm, speed=25):
        """Drive straight with PID gyro correction"""
        start_angle = self.gyro.angle
        rotations = distance_cm / (7.0 * 3.14159)  # Updated to 7cm wheel diameter
        
        print("Driving {}cm with heading {} degrees".format(distance_cm, start_angle))
        
        # Reset PID values
        self.integral = 0
        self.last_error = 0
        
        # Motor direction: motors are reversed, so positive distance needs negative speed
        direction = -1 if distance_cm >= 0 else 1  # Negative because motors are reversed
        abs_speed = abs(speed)
        
        self.tank_drive.on_for_rotations(
            direction * abs_speed, 
            direction * abs_speed, 
            abs(rotations), 
            block=False
        )
        
        while self.tank_drive.is_running:
            current_angle = self.gyro.angle
            error = self.normalize_angle(current_angle - start_angle)
            
            # PID calculation
            self.integral = max(-50, min(50, self.integral + error))
            derivative = error - self.last_error
            
            correction = (error * self.KP + 
                        self.integral * self.KI + 
                        derivative * self.KD)
            
            left_speed = direction * abs_speed - correction
            right_speed = direction * abs_speed + correction
            
            # Limit speeds
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
            
            self.left_motor.on(SpeedPercent(left_speed))
            self.right_motor.on(SpeedPercent(right_speed))
            
            print("Angle: {} degrees, Error: {} degrees, Correction: {}".format(
                current_angle, error, correction))
            
            self.last_error = error
            sleep(0.01)
            
        self.tank_drive.off()
        print("Drive complete. Final angle: {} degrees".format(self.gyro.angle))
    
    def drive_square(self, side_length_cm=30, repeats=1):
        """Drive a square with the given side length"""
        print("\nStarting square drive with side length {}cm".format(side_length_cm))
        
        for i in range(repeats):
            print("\n=== Square {} ===".format(i+1))
            for j in range(4):
                print("\n--- Side {} ---".format(j+1))
                # Drive straight
                self.drive_straight(side_length_cm)
                sleep(0.5)  # Small pause between driving and turning
                
                # Turn 90 degrees
                current_angle = self.gyro.angle
                self.turn_to_angle(current_angle + 90)
                sleep(0.5)  # Small pause between turning and next side
        
        print("\nSquare drive complete!")

if __name__ == "__main__":
    print("Square Test - Starting...")
    robot = SquareTest()
    
    try:
        while True:
            input("Press ENTER to drive a square (Ctrl+C to exit)...")
            robot.drive_square(30, repeats=1)  # Drive square with 30cm sides
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        # Ensure motors are stopped
        robot.tank_drive.off()
