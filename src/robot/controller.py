# -*- coding: utf-8 -*-
"""
Robot controller that handles motors and basic operations (no sensors)
"""

from ev3dev2.motor import LargeMotor, MoveTank, MediumMotor
from ev3dev2.button import Button
from time import sleep
from ..config.settings import *
import threading

class RobotController:
    
    def __init__(self):
        print("Initializing robot controller...")
        
        # Initialize motors (ports A and D)
        self.left_motor = LargeMotor('outA')
        self.right_motor = LargeMotor('outD')
        self.tank_drive = MoveTank('outA', 'outD')

        # Medium motor for collect mechanism
        self.collect_motor = MediumMotor('outC')

        self.monitoring = True
        self.blockage_thread = threading.Thread(target=self._monitor_harvester_blockage, daemon= True)
        self.blockage_thread.start()

        
        # Re-enabled: Harvester motor needed for ball collection
        try:
            self.collect_motor.on(speed=-100)  # FASTER: Increased collect speed from -30 to -50
            print("Collect mechanism started (Port C) - speed -50")
        except Exception as e:
            print("Failed to start collect mechanism:", e)

        # Initialize button (if available)
        try:
            self.button = Button()
        except:
            self.button = None
            print("No button available")
        
        # State
        self.running = True
        
        print("Robot controller initialized")
    
    def stop_all_motors(self):
        """Emergency stop for all motors"""
        try:
            self.tank_drive.off()
            self.left_motor.stop()
            self.right_motor.stop()
            self.collect_motor.off()
         
            print("All motors stopped")
        except Exception as e:
            print("Error stopping motors: {}".format(e))
    
    def check_buttons(self):
        """Checks for BACK button press to stop the robot"""
        if self.button and self.button.backspace:
            print("Back button pressed - stopping robot")
            self.running = False
            self.stop_all_motors()
            return True
        return False
    
    def normalize_angle(self, angle):
        """Normalizes angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle 
    
    def get_current_heading(self):
        """Returns current direction - no gyro, so returns 0"""
        return 0  # No gyro - only vision navigation
    
    def simple_turn(self, direction, angle_degrees):
        """
        Turn the robot by a given angle
        Minimum angle: 5 degrees
        """
        speed = ROBOT_TURN_SPEED
        
        # Minimum angle threshold - EV3 motors cannot reliably turn under ~5 degrees
        min_angle = 2.0
        actual_angle = max(abs(angle_degrees), min_angle)
        
        # Calculate rotations based on angle
        # 0.5 rotations = 90 degrees, so rotations = (angle / 90) * 0.5
        rotations = actual_angle / 90.0 * 0.5
        
        print("Simple turn: {} {:.1f} degrees -> {:.1f} degrees ({:.3f} rotations)".format(
            direction, angle_degrees, actual_angle, rotations))
        
        # Skip if angle is too small
        if rotations < 0.01:  # Under 0.05 rotations = no movement
            print("Angle too small, skipping turn")
            return
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motors are reversed)
            self.tank_drive.on_for_rotations(speed, -speed, rotations)
        else:
            # Turn left: left motor backward, right motor forward (motors are reversed)  
            self.tank_drive.on_for_rotations(-speed, speed, rotations)
            
        print("Simple turn complete")
    
    def simple_turn_rotation(self, direction, rotations):
        """
        Turn the robot with direct rotations (NO angle conversion)
        rotations: number of rotations (0.5 = 90°, 1.0 = 180°)
        """
        speed = ROBOT_TURN_SPEED
        
        print("Simple turn rotation: {} {:.3f} rotations (DIRECT - no angle conversion)".format(
            direction, rotations))
        
        # Skip if rotation is too small - reduced for better precision
        if rotations < 0.0000001:  # Reduced from 0.01 to 0.003 for fine-tuning
            print("Rotation too small, skipping turn")
            return
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motors are reversed)
            self.tank_drive.on_for_rotations(speed, -speed, rotations)
        else:
            # Turn left: left motor backward, right motor forward (motors are reversed)  
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
        Wheel diameter: 68.8 mm = 6.88 cm → circumference: ~21.6 cm per rotation
        """
        speed = ROBOT_FORWARD_SPEED
        
        # Calculate total distance including overshoot (now 0 as default)
        total_distance = distance_cm + overshoot_cm
        
        # Calculate rotations based on wheel diameter
        wheel_diameter_cm = 6.88  # 68.8 mm = 6.88 cm correct measurement of wheel diameter  
        wheel_circumference_cm = 3.14159 * wheel_diameter_cm  # π × diameter ≈ 21.6 cm
        rotations = total_distance / wheel_circumference_cm
        
        if overshoot_cm > 0:
            print("Simple forward: {:.1f} cm + {:.1f} cm overshoot = {:.1f} cm total ({:.3f} rotations)".format(
                distance_cm, overshoot_cm, total_distance, rotations))
        else:
            print("Simple forward: {:.1f} cm ({:.3f} rotations) - NO OVERSHOOT".format(
                distance_cm, rotations))
        
        # Both motors forward (motors are reversed, so use negative values)
        self.tank_drive.on_for_rotations(-speed, -speed, rotations)
        
        print("Simple forward complete")
    


    def simple_backward(self, distance_cm):
        """
        Move robot backward using motor rotations
        """
        speed = ROBOT_FORWARD_SPEED
        
        # Calculate rotations based on wheel diameter  
        wheel_diameter_cm = 6.88  # 68.8 mm = 6.88 cm correct measurement of wheel diameter
        wheel_circumference_cm = 3.14159 * wheel_diameter_cm  # π × diameter ≈ 21.6 cm
        rotations = distance_cm / wheel_circumference_cm
        
        print("Simple backward: {:.1f} cm ({:.3f} rotations)".format(distance_cm, rotations))
        
        # Both motors backward (motors are reversed, so use positive values)
        self.tank_drive.on_for_rotations(speed, speed, rotations)
        
        print("Simple backward complete")

    def start_continuous_move(self, speed):
        """Starts continuous movement of the robot at a given speed.
        Negative speed for forward, positive for backward (based on observed behavior).
        """
        # Use negative speed for forward movement based on user's confirmation
        self.tank_drive.on(left_speed=-speed, right_speed=-speed)
        print("Robot started continuous move with speed: {}".format(speed))

    def stop_continuous_move(self):
        """Stops any continuous movement by turning off tank drive."""
        self.tank_drive.off()
        print("Robot stopped continuous move")

    def turn_180_degrees(self):
        """
        Precise 180 degree turn based on motor rotations
        180 degrees = 1 rotation with wheels in opposite directions = 1.0 rotation
        """
        print("Turning 180 degrees using direct rotation method")
        self.simple_turn_rotation("right", 1.0)  # 1.0 rotation = 180°
        print("180 degree turn complete")
    
    def blind_ball_collection(self):
        """
        BLIND BALL COLLECTION SEQUENCE:
        1. Drive 20 cm straight out (ca 1 rotation)
        2. Back up 20 cm (ca 1 rotation) 
        3. Rotate 180 degrees (ca 1 rotation in each direction)
        """
        print("*** BLIND BALL COLLECTION ***")
        
        # Step 1: Drive 20 cm forward (blind - doesn't matter if ball disappears)
        print("STEP 1: Driving forward 20 cm (BLIND)")
        self.simple_forward(5, overshoot_cm=0)  # No overshoot - exactly 20 cm
        
        # Step 2: Back up 20 cm
        print("STEP 2: Backing up 20 cm")
        self.simple_backward(20)
        
        # Step 3: Rotate 180 degrees (ca 1 rotation in each direction)
        print("STEP 3: Rotating 180 degrees for next ball")
        # self.turn_180_degrees()
        
        print("*** BLIND BALL COLLECTION COMPLETE ***")


    def release_balls(self, duration=4):
        """
        Run collect_motor in opposite direction to release balls.
        duration: how long the motor should run (seconds)
        """
        print("Releasing balls")
        try:
            self.collect_motor.on(speed=60)  # Run forward (opposite of collection)
            sleep(3)
            self.collect_motor.on(speed=-60)  # Run forward (opposite of collection)
            sleep(0.5)
            self.collect_motor.on(speed=60)  # Run forward (opposite of collection)
            sleep(3)
            self.collect_motor.on(speed=-60)  # Run forward (opposite of collection)
            sleep(0.5)
            print("Balls released")
            self.simple_backward(10) # Move backward 10 cm after releasing balls
            print("Backed up 10 cm after ball release")
        except Exception as e:
            print("Error releasing balls:", e)

    def _monitor_harvester_blockage(self):
        print("MONITORING STALL")
        while self.monitoring:
            try:
                if self.collect_motor.is_stalled:
                    print("Harvester blocked! Moving backwards")
                    self.simple_backward(10)
                    #Restart the harvester motor
                    self.collect_motor.on(speed=-100)
                    print("Harvester restarted after blockage")
            except Exception as e:
                print("Error in blockage monitor")
            sleep(0.2) #Check 5 times per second
    
    def stop_monitoring(self):
        self.monitoring = False

    def cleanup(self):
        """Shuts down the robot and stops all motors"""
        print("Cleaning up robot controller...")
        self.stop_all_motors()
        self.running = False 

    