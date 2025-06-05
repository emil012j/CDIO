#!/usr/bin/env python3 

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1
import socket
import json
import math
from time import sleep

class RobotController:
    def __init__(self):
        # Motorer - bruger A og D
        self.tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        
        # Kun gyro sensor
        self.gyro = GyroSensor(INPUT_1)
        
        # Netværk
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.command_socket.bind(('', 1233))
        self.command_socket.listen(1)
        
        # Kalibrer gyro ved start
        self.reset_gyro()
        
    def reset_gyro(self):
        """Nulstil og kalibrer gyro sensor"""
        self.gyro.mode = 'GYRO-CAL'
        sleep(0.2)
        self.gyro.mode = 'GYRO-ANG'
        sleep(0.2)
        
    def turn_to_angle(self, target_angle, speed=20):
        """Drej til specifik vinkel med gyro-præcision"""
        current_angle = self.gyro.angle
        angle_to_turn = target_angle - current_angle
        
        # Normaliser vinkel til [-180, 180]
        while angle_to_turn > 180:
            angle_to_turn -= 360
        while angle_to_turn < -180:
            angle_to_turn += 360
            
        # Præcis drejning med aftagende hastighed
        while abs(self.gyro.angle - target_angle) > 2:  # 2 graders tolerance
            remaining = abs(target_angle - self.gyro.angle)
            current_speed = min(speed, max(10, remaining * 0.5))
            
            if angle_to_turn > 0:
                self.tank_drive.on(-current_speed, current_speed)
            else:
                self.tank_drive.on(current_speed, -current_speed)
                
        self.tank_drive.off()
        
    def drive_straight(self, distance_cm, speed=30):
        """Kør lige ud med gyro-korrektion"""
        start_angle = self.gyro.angle
        rotations = distance_cm / (5.6 * math.pi)  # Juster 5.6 til din hjuldiameter
        
        self.tank_drive.on_for_rotations(speed, speed, rotations, block=False)
        
        while self.tank_drive.is_running:
            current_angle = self.gyro.angle
            error = current_angle - start_angle
            correction = error * 0.5
            
            self.left_motor.on(SpeedPercent(speed - correction))
            self.right_motor.on(SpeedPercent(speed + correction))
            sleep(0.01)
            
        self.tank_drive.off()
        
    def navigate_to_coordinate(self, target_x, target_y, current_x, current_y):
        """Naviger direkte til koordinat"""
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.degrees(math.atan2(dy, dx))
        
        print(f"Navigerer til ({target_x}, {target_y}) fra ({current_x}, {current_y})")
        print(f"Drejer til vinkel: {target_angle}")
        
        # Drej mod målet
        self.turn_to_angle(target_angle)
        
        # Beregn afstand og kør
        distance = math.sqrt(dx*dx + dy*dy)
        print(f"Kører {distance:.1f} cm")
        self.drive_straight(distance)
        
    def run(self):
        """Hovedloop der lytter efter kommandoer"""
        print("Robot starter og venter på kommandoer...")
        
        while True:
            conn, addr = self.command_socket.accept()
            try:
                data = conn.recv(1024).decode()
                command = json.loads(data)
                
                if 'coordinates' in command:
                    coords = command['coordinates']
                    print(f"Modtog koordinater: {coords}")
                    self.navigate_to_coordinate(
                        coords['target_x'],
                        coords['target_y'],
                        coords['current_x'],
                        coords['current_y']
                    )
                elif 'direction' in command:
                    direction = command['direction']
                    distance = command.get('distance', 0)
                    print(f"Modtog retning: {direction}, distance: {distance}")
                    
                    if direction == "FORWARD":
                        self.drive_straight(distance)
                    elif direction == "BACKWARD":
                        self.drive_straight(-distance)
                    elif direction == "RIGHT":
                        self.turn_to_angle(self.gyro.angle + 90)
                    elif direction == "LEFT":
                        self.turn_to_angle(self.gyro.angle - 90)
                
            except Exception as e:
                print(f"Fejl under kommando behandling: {e}")
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