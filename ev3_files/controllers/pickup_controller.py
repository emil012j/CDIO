#!/usr/bin/env python3

from ev3dev2.motor import MediumMotor, OUTPUT_A
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sound import Sound
import time

class PickupController:
    """Controls the ball pickup mechanism on the EV3 robot"""
    
    def __init__(self, motor_port=OUTPUT_A, touch_sensor_port=INPUT_1, color_sensor_port=INPUT_4):
        """Initialize the pickup controller with the specified ports"""
        self.initialized = False
        self.sound = Sound()
        
        try:
            # Initialize pickup motor
            self.pickup_motor = MediumMotor(motor_port)
            self.pickup_motor.reset()
            
            # Initialize sensors if specified
            self.has_touch_sensor = False
            self.has_color_sensor = False
            
            try:
                self.touch_sensor = TouchSensor(touch_sensor_port)
                self.has_touch_sensor = True
            except Exception as e:
                print(f"Touch sensor not available: {e}")
                
            try:
                self.color_sensor = ColorSensor(color_sensor_port)
                self.has_color_sensor = True
            except Exception as e:
                print(f"Color sensor not available: {e}")
                
            # Set default values
            self.pickup_speed = 50
            self.initialized = True
            print("Pickup controller initialized successfully")
            self.sound.beep()
            
        except Exception as e:
            print(f"Failed to initialize pickup controller: {e}")
    
    def pickup(self, duration=1.0):
        """Activate the pickup mechanism to grab a ball"""
        if not self.initialized:
            return False
            
        try:
            # Run pickup motor
            self.pickup_motor.on(self.pickup_speed)
            
            if self.has_touch_sensor:
                # Wait for touch sensor to be triggered or timeout
                start_time = time.time()
                while not self.touch_sensor.is_pressed and (time.time() - start_time) < duration:
                    time.sleep(0.05)
                    
                # Stop motor when touch sensor triggered or timeout
                self.pickup_motor.off()
                
                # Return success based on touch sensor
                return self.touch_sensor.is_pressed
            else:
                # No touch sensor, just run for the specified duration
                time.sleep(duration)
                self.pickup_motor.off()
                return True
                
        except Exception as e:
            print(f"Error during pickup: {e}")
            self.pickup_motor.off()
            return False
    
    def release(self, duration=1.0):
        """Release a ball by reversing the pickup mechanism"""
        if not self.initialized:
            return False
            
        try:
            # Reverse pickup motor
            self.pickup_motor.on(-self.pickup_speed)
            time.sleep(duration)
            self.pickup_motor.off()
            return True
            
        except Exception as e:
            print(f"Error during release: {e}")
            self.pickup_motor.off()
            return False
    
    def detect_ball(self):
        """
        Detect if a ball is in front of the robot using color sensor
        Returns ball color or None if no ball detected
        """
        if not self.initialized or not self.has_color_sensor:
            return None
            
        try:
            # Get color from sensor
            color = self.color_sensor.color_name
            
            # Check if color indicates a ball
            if color in ['WHITE', 'YELLOW', 'RED', 'ORANGE']:
                return color
            return None
            
        except Exception as e:
            print(f"Error detecting ball: {e}")
            return None
    
    def stop(self):
        """Stop the pickup motor"""
        if not self.initialized:
            return False
            
        try:
            self.pickup_motor.off()
            return True
        except Exception as e:
            print(f"Error stopping pickup motor: {e}")
            return False

if __name__ == "__main__":
    # Test pickup controller
    controller = PickupController()
    
    if controller.initialized:
        print("Testing pickup mechanism...")
        controller.pickup(duration=1)
        time.sleep(1)
        controller.release(duration=1)
        
        # Test ball detection if color sensor available
        if controller.has_color_sensor:
            print("Testing ball detection...")
            color = controller.detect_ball()
            if color:
                print(f"Ball detected: {color}")
            else:
                print("No ball detected")
                
        print("Pickup test complete")