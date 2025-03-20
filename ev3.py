#!/usr/bin/env python3 

from time import sleep
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor, InfraredSensor 
us = UltrasonicSensor()
colorsensor = ColorSensor()
ir = InfraredSensor()
while True: 
    distance = us.distance_centimeters 
    print("Distance: {} cm".format(distance))
    sleep(0.5)


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