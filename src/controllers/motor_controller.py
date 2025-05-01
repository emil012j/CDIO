#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
import math
import time

class MotorController:
    """Controls the motors for an EV3 robot with differential drive"""
    
    def __init__(self, left_motor_port=OUTPUT_B, right_motor_port=OUTPUT_C):
        """Initialize motor controller with specified motor ports"""
        try:
            self.left_motor = LargeMotor(left_motor_port)
            self.right_motor = LargeMotor(right_motor_port)
            self.tank_drive = MoveTank(left_motor_port, right_motor_port)
            self.sound = Sound()
            self.leds = Leds()
            
            # Confirm initialization
            self.sound.beep()
            self.leds.set_color("LEFT", "GREEN")
            self.leds.set_color("RIGHT", "GREEN")
            
            # Motor settings
            self.left_motor.reset()
            self.right_motor.reset()
            self.default_speed = 50
            self.turning_speed = 30
            self.initialized = True
            
            print("Motor controller initialized successfully")
        except Exception as e:
            print(f"Failed to initialize motor controller: {e}")
            self.initialized = False
    
    def move_forward(self, speed=None, duration=None):
        """Move robot forward at specified speed for optional duration"""
        if not self.initialized:
            return False
        
        speed = speed if speed is not None else self.default_speed
        
        try:
            self.tank_drive.on(speed, speed)
            
            if duration is not None:
                time.sleep(duration)
                self.stop()
                
            return True
        except Exception as e:
            print(f"Error moving forward: {e}")
            return False
    
    def move_backward(self, speed=None, duration=None):
        """Move robot backward at specified speed for optional duration"""
        if not self.initialized:
            return False
        
        speed = speed if speed is not None else self.default_speed
        
        try:
            self.tank_drive.on(-speed, -speed)
            
            if duration is not None:
                time.sleep(duration)
                self.stop()
                
            return True
        except Exception as e:
            print(f"Error moving backward: {e}")
            return False
    
    def turn_left(self, speed=None, duration=None):
        """Turn robot left at specified speed for optional duration"""
        if not self.initialized:
            return False
        
        speed = speed if speed is not None else self.turning_speed
        
        try:
            self.tank_drive.on(-speed, speed)
            
            if duration is not None:
                time.sleep(duration)
                self.stop()
                
            return True
        except Exception as e:
            print(f"Error turning left: {e}")
            return False
    
    def turn_right(self, speed=None, duration=None):
        """Turn robot right at specified speed for optional duration"""
        if not self.initialized:
            return False
        
        speed = speed if speed is not None else self.turning_speed
        
        try:
            self.tank_drive.on(speed, -speed)
            
            if duration is not None:
                time.sleep(duration)
                self.stop()
                
            return True
        except Exception as e:
            print(f"Error turning right: {e}")
            return False
    
    def rotate_to_angle(self, target_angle, current_angle, speed=None):
        """Rotate the robot to align with a specific angle"""
        if not self.initialized:
            return False
        
        speed = speed if speed is not None else self.turning_speed
        
        # Normalize angles to -180 to 180 range
        current_angle = ((current_angle + 180) % 360) - 180
        target_angle = ((target_angle + 180) % 360) - 180
        
        # Calculate shortest rotation direction
        angle_diff = target_angle - current_angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        # Determine turn direction and execute
        if abs(angle_diff) < 3:
            # Already close enough to target angle
            self.stop()
            return True
        elif angle_diff > 0:
            # Turn right
            rotation_speed = min(speed, max(15, abs(angle_diff) / 3))
            self.tank_drive.on(rotation_speed, -rotation_speed)
        else:
            # Turn left
            rotation_speed = min(speed, max(15, abs(angle_diff) / 3))
            self.tank_drive.on(-rotation_speed, rotation_speed)
            
        return False  # Indicates rotation is still in progress
    
    def move_to_coordinates(self, current_x, current_y, current_angle, target_x, target_y, precision=20):
        """
        Move toward target coordinates based on current position and orientation
        Returns action taken as string
        """
        if not self.initialized:
            return "ERROR"
            
        # Calculate distance to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If we're close enough to target, stop
        if distance < precision:
            self.stop()
            return "TARGET_REACHED"
            
        # Calculate angle to target in degrees
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Normalize angles to -180 to 180 range
        current_angle = ((current_angle + 180) % 360) - 180
        target_angle = ((target_angle + 180) % 360) - 180
        
        # Calculate angle difference
        angle_diff = target_angle - current_angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        # Decide action based on angle difference
        if abs(angle_diff) > 20:
            # Need to rotate first
            if angle_diff > 0:
                self.turn_right()
                return "TURNING_RIGHT"
            else:
                self.turn_left()
                return "TURNING_LEFT"
        else:
            # Heading is good enough, move forward
            # Adjust speed based on distance to target
            speed = min(self.default_speed, max(20, distance / 5))
            self.move_forward(speed)
            return "MOVING_FORWARD"
    
    def stop(self):
        """Stop all motors"""
        if not self.initialized:
            return False
            
        try:
            self.tank_drive.off()
            return True
        except Exception as e:
            print(f"Error stopping motors: {e}")
            return False

if __name__ == "__main__":
    # Test motor controller
    controller = MotorController()
    
    if controller.initialized:
        print("Testing motor movements...")
        controller.move_forward(duration=1)
        time.sleep(0.5)
        controller.move_backward(duration=1)
        time.sleep(0.5)
        controller.turn_left(duration=1)
        time.sleep(0.5)
        controller.turn_right(duration=1)
        time.sleep(0.5)
        controller.stop()
        print("Motor test complete")