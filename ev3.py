#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1
import socket
import json
import math
from time import sleep

class RobotController:
    def __init__(self):
        # Motors - A (left) and D (right), motors are reversed (minus = forward)
        self.tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        
        # Gyro sensor
        self.gyro = GyroSensor(INPUT_1)
        
        # Network
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.command_socket.bind(('', 1233))
            self.command_socket.listen(1)
            print("Command server listening on port 1233...")
        except Exception as e:
            print("Error binding to port 1233: {}".format(e))
        
        # PID constants for straight driving - optimized for 7cm wheels
        self.KP = 0.5  # Reduced for less aggressive correction
        self.KI = 0.01  # Reduced to avoid overstyring
        self.KD = 0.3  # Increased for better damping
        self.integral = 0
        self.last_error = 0
        
        # Calibrate gyro at start
        print("Calibrating gyro...")
        self.reset_gyro()
        print("Gyro calibrated. Start angle: {}".format(self.gyro.angle))
        
    def reset_gyro(self):
        """Reset and calibrate gyro sensor"""
        self.gyro.mode = 'GYRO-CAL'
        sleep(0.2)
        self.gyro.mode = 'GYRO-ANG'
        sleep(0.2)
        self.integral = 0
        self.last_error = 0
        
    def turn_to_angle(self, target_angle, speed=15):
        """Turn to specific angle with gyro precision"""
        current_angle = self.gyro.angle
        angle_to_turn = target_angle - current_angle
        
        print("Turning from {} to {} (need to turn {})".format(
            current_angle, target_angle, angle_to_turn))
        
        # Normalize angle to [-180, 180]
        while angle_to_turn > 180:
            angle_to_turn -= 360
        while angle_to_turn < -180:
            angle_to_turn += 360
            
        # Precise turning with decreasing speed
        while abs(self.gyro.angle - target_angle) > 1:  # 1 degree tolerance
            remaining = abs(target_angle - self.gyro.angle)
            current_speed = min(speed, max(5, remaining * 0.4))
            
            if angle_to_turn > 0:  # Turn right
                self.tank_drive.on(current_speed, -current_speed)  # Motors are reversed
            else:  # Turn left
                self.tank_drive.on(-current_speed, current_speed)  # Motors are reversed
            
            print("Angle: {}, Remaining: {}".format(self.gyro.angle, remaining))
                
        self.tank_drive.off()
        print("Turn complete. Final angle: {}".format(self.gyro.angle))
        
    def normalize_angle(self, angle):
        """Normalize angle to range [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
        
    def navigate_to_coordinate(self, target_x, target_y, current_x, current_y):
        """Navigate directly to coordinate"""
        # Coordinates come in mm, convert to cm for calculation
        target_x = target_x / 10.0
        target_y = target_y / 10.0
        current_x = current_x / 10.0
        current_y = current_y / 10.0
        
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Calculate target angle and normalize it
        target_angle = math.degrees(math.atan2(dy, dx))
        target_angle = self.normalize_angle(target_angle)
        
        # Calculate shortest turn from current angle
        current_angle = self.normalize_angle(self.gyro.angle)
        angle_diff = self.normalize_angle(target_angle - current_angle)
        
        print("\nNew navigation:")
        print("From position: ({:.1f}, {:.1f})".format(current_x, current_y))
        print("To position: ({:.1f}, {:.1f})".format(target_x, target_y))
        print("Current angle: {}, Target angle: {}, Need to turn: {}".format(
            current_angle, target_angle, angle_diff))
        
        # Turn towards target
        self.turn_to_angle(target_angle)
        
        # Calculate distance and drive
        distance = math.sqrt(dx*dx + dy*dy)
        print("Driving {:.1f} cm".format(distance))
        self.drive_straight(distance)
        
    def drive_straight(self, distance_cm, speed=25):
        """Drive straight with PID gyro correction"""
        start_angle = self.gyro.angle
        rotations = distance_cm / (7.0 * math.pi)  # Updated to 7cm wheel diameter
        
        print("Driving {}cm with heading {}".format(distance_cm, start_angle))
        
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
            self.integral = max(-50, min(50, self.integral + error))  # Limit integral
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
            
            print("Angle: {}, Error: {}, Correction: {}".format(
                current_angle, error, correction))
            
            self.last_error = error
            sleep(0.01)
            
        self.tank_drive.off()
        print("Drive complete. Final angle: {}".format(self.gyro.angle))
        
    def simple_turn(self, direction, duration):
        """Simple turn without gyro - just time-based"""
        speed = 40  # Faster turning speed
        print("Simple turn: {} for {:.2f} seconds".format(direction, duration))
        
        if direction == "right":
            # Turn right: left motor forward, right motor backward (motors are reversed)
            self.tank_drive.on(speed, -speed)
        else:
            # Turn left: left motor backward, right motor forward (motors are reversed)  
            self.tank_drive.on(-speed, speed)
            
        sleep(duration)
        self.tank_drive.off()
        print("Simple turn complete")
        
    def simple_forward(self, distance_cm):
        """Simple forward movement without gyro - just time-based"""
        speed = 50  # Faster forward speed
        # Estimate time based on distance (rough calibration needed)
        duration = distance_cm / 20.0  # Faster - roughly 20cm per second
        
        print("Simple forward: {}cm for {:.2f} seconds".format(distance_cm, duration))
        
        # Both motors forward (negative because motors are reversed)
        self.tank_drive.on(-speed, -speed)
        sleep(duration)
        self.tank_drive.off()
        print("Simple forward complete")
        
    def run(self):
        """Main loop that listens for commands"""
        print("Robot starting and waiting for commands...")
        
        while True:
            conn, addr = self.command_socket.accept()
            try:
                data = conn.recv(1024).decode()
                command = json.loads(data)
                
                if 'command' in command:
                    cmd_type = command['command']
                    print("\nReceived command: {}".format(command))
                    
                    if cmd_type == "simple_turn":
                        direction = command.get('direction', 'right')
                        duration = command.get('duration', 1.0)
                        print("Simple turn {} for {:.2f} seconds".format(direction, duration))
                        self.simple_turn(direction, duration)
                    elif cmd_type == "turn":
                        angle = command.get('angle', 0)
                        print("Turning {} degrees".format(angle))
                        target_angle = self.normalize_angle(self.gyro.angle + angle)
                        self.turn_to_angle(target_angle)
                    elif cmd_type == "forward":
                        distance = command.get('distance', 0)
                        print("Moving forward {} cm".format(distance))
                        self.simple_forward(distance)
                        
                elif 'coordinates' in command:
                    coords = command['coordinates']
                    print("\nReceived coordinates: {}".format(coords))
                    self.navigate_to_coordinate(
                        coords['target_x'],
                        coords['target_y'],
                        coords['current_x'],
                        coords['current_y']
                    )
                elif 'direction' in command:
                    direction = command['direction']
                    distance = command.get('distance', 0)
                    print("\nReceived direction: {}, distance: {}".format(direction, distance))
                    
                    if direction == "FORWARD":
                        self.drive_straight(distance)
                    elif direction == "BACKWARD":
                        self.drive_straight(-distance)
                    elif direction == "RIGHT":
                        self.turn_to_angle(self.gyro.angle + 90)
                    elif direction == "LEFT":
                        self.turn_to_angle(self.gyro.angle - 90)
                
            except Exception as e:
                print("Error during command handling: {}".format(e))
            finally:
                conn.close()

if __name__ == "__main__":
    robot = RobotController()
    robot.run()


"""
# Opsæt de store motorer på OUTPUT_A og OUTPUT_B
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Opsæt mellem motoren (armen) på OUTPUT_C
motor_c = MediumMotor(OUTPUT_C)


while True:
    distance = us.distance_centimeters
    reflected_light = colorsensor.reflected_light_intensity
    ir_distance = ir.proximity 
    # Bruger ultralydssensor til at mærke om der er vægge 
    if distance < 10 :
        motor_a.on(SpeedPercent(-50))
        motor_b.on(SpeedPercent(50))
        sleep(2)

    # Bruger farvesensor til at aflæse om der er en bold der er taget
    elif reflected_light > 80 :                        # Belysning på meget lys farve (bold er højst sandsynligt meget lys eller hvid)
        motor_c.on_for_degrees(SpeedPercent(60), 45)   # Frem
        motor_c.on_for_degrees(SpeedPercent(-60), 45)  # Tilbage
    
    #Bruger IR sensor til at aflæse om der er en 
    if ir_distance < 4 :
        motor_a.off()
        motor_b.off() 
        sleep(2)

"""