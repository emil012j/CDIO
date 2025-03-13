#!/usr/bin/env python3 

from time import sleep
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor.lego import ColorSensor
us = UltrasonicSensor()

colorsensor = ColorSensor()

# Opsæt de store motorer på OUTPUT_A og OUTPUT_B
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Opsæt mellem motoren (armen) på OUTPUT_C
motor_c = MediumMotor(OUTPUT_C)

# Start med at køre hurtigt fremad i 1,5 sekunder
motor_a.on(SpeedPercent(100))
motor_b.on(SpeedPercent(100))
sleep(3)

# Drej skarpt til højre
motor_a.on(SpeedPercent(100))
motor_b.on(SpeedPercent(-50))
sleep(1)

# Bak hurtigt tilbage
motor_a.on(SpeedPercent(-80))
motor_b.on(SpeedPercent(-80))
sleep(1)

# Drej rundt på stedet
motor_a.on(SpeedPercent(100))
motor_b.on(SpeedPercent(-100))
sleep(1)

# Stop de store motorer
motor_a.off()
motor_b.off()

# Kør armen en halv omgang opad
motor_c.on_for_degrees(SpeedPercent(75), 90)

# Vift armen frem og tilbage hurtigt 3 gange 
for _ in range(3):
    motor_c.on_for_degrees(SpeedPercent(60), 45)   # Frem
    motor_c.on_for_degrees(SpeedPercent(-60), 45)  # Tilbage

# Stop mellem motoren og gå til neutral position
motor_c.on_for_degrees(SpeedPercent(-50), 45)
motor_c.off()


while True:
    distance = us.distance_centimeters
    reflected_light = colorsensor.reflected_light_intensity
    # Bruger ultralydssensor til at mærke om der er vægge 
    if distance < 10 :
        motor_a.on(SpeedPercent(-50))
        motor_b.on(SpeedPercent(50))
        sleep(1)

    # Bruger farvesensor til at aflæse om der er en bold der er taget
    elif reflected_light > 80 :                        # Belysning på meget lys farve (bold er højst sandsynligt meget lys eller hvid)
        motor_c.on_for_degrees(SpeedPercent(60), 45)   # Frem
        motor_c.on_for_degrees(SpeedPercent(-60), 45)  # Tilbage
       


