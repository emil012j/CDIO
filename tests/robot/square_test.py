#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1
from time import sleep

class SquareTest:
    def __init__(self):
        # Motorer
        self.tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        
        # Gyro sensor
        self.gyro = GyroSensor(INPUT_1)
        
        # PID konstanter til lige korsel
        self.KP = 1.0
        self.KI = 0.02
        self.KD = 0.2
        self.integral = 0
        self.last_error = 0
        
        # Kalibrer gyro ved start
        print("Kalibrerer gyro...")
        self.reset_gyro()
        print("Gyro kalibreret. Start vinkel: {}".format(self.gyro.angle))
    
    def reset_gyro(self):
        """Nulstil og kalibrer gyro sensor"""
        self.gyro.mode = "GYRO-CAL"
        sleep(0.2)
        self.gyro.mode = "GYRO-ANG"
        sleep(0.2)
        self.integral = 0
        self.last_error = 0
    
    def normalize_angle(self, angle):
        """Normaliser vinkel til intervallet [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def turn_to_angle(self, target_angle, speed=20):
        """Drej til specifik vinkel med gyro-praecision"""
        current_angle = self.gyro.angle
        angle_to_turn = self.normalize_angle(target_angle - current_angle)
        
        print("Drejer fra {} grader til {} grader (skal dreje {} grader)".format(
            current_angle, target_angle, angle_to_turn))
        
        # Praecis drejning med aftagende hastighed
        while abs(self.gyro.angle - target_angle) > 2:  # 2 graders tolerance
            remaining = abs(target_angle - self.gyro.angle)
            current_speed = min(speed, max(10, remaining * 0.5))
            
            if angle_to_turn > 0:  # Drej til højre
                self.tank_drive.on(current_speed, -current_speed)  # Vendt om pga. motormontering
            else:  # Drej til venstre
                self.tank_drive.on(-current_speed, current_speed)  # Vendt om pga. motormontering
            
            print("Vinkel: {} grader, Mangler: {} grader".format(
                self.gyro.angle, remaining))
                
        self.tank_drive.off()
        print("Drejning faerdig. Slut vinkel: {} grader".format(self.gyro.angle))
    
    def drive_straight(self, distance_cm, speed=30):
        """Kor lige ud med PID gyro-korrektion"""
        start_angle = self.gyro.angle
        rotations = distance_cm / (5.6 * 3.14159)  # Juster 5.6 til din hjuldiameter
        
        print("Korer {}cm med kurs {} grader".format(distance_cm, start_angle))
        
        # Nulstil PID vaerdier
        self.integral = 0
        self.last_error = 0
        
        # Vend motorretning baseret paa om distance er positiv eller negativ
        direction = -1 if distance_cm >= 0 else 1  # Vendt om pga. motormontering
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
            
            # PID beregning
            self.integral = max(-50, min(50, self.integral + error))
            derivative = error - self.last_error
            
            correction = (error * self.KP + 
                        self.integral * self.KI + 
                        derivative * self.KD)
            
            left_speed = direction * abs_speed - correction
            right_speed = direction * abs_speed + correction
            
            # Begraens hastigheder
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
            
            self.left_motor.on(SpeedPercent(left_speed))
            self.right_motor.on(SpeedPercent(right_speed))
            
            print("Vinkel: {} grader, Fejl: {} grader, Korrektion: {}".format(
                current_angle, error, correction))
            
            self.last_error = error
            sleep(0.01)
            
        self.tank_drive.off()
        print("Korsel faerdig. Slut vinkel: {} grader".format(self.gyro.angle))
    
    def drive_square(self, side_length_cm=30):
        """Kor en firkant med den givne sidelaengde"""
        print("\nStarter firkant-korsel med sidelaengde {}cm".format(side_length_cm))
        
        for i in range(4):
            print("\n--- Side {} ---".format(i+1))
            # Kor lige ud
            self.drive_straight(side_length_cm)
            sleep(0.5)  # Lille pause mellem korsel og drejning
            
            # Drej 90 grader
            current_angle = self.gyro.angle
            self.turn_to_angle(current_angle + 90)
            sleep(0.5)  # Lille pause mellem drejning og naeste side
        
        print("\nFirkant-korsel faerdig!")

if __name__ == "__main__":
    print("Square Test - Starter...")
    robot = SquareTest()
    
    try:
        while True:
            input("Tryk ENTER for at kore en firkant...")
            robot.drive_square(30)  # Kor firkant med 30cm sider
    except KeyboardInterrupt:
        print("\nProgram afbrudt af bruger")
    finally:
        # Sikr at motorerne stoppes
        robot.tank_drive.off()