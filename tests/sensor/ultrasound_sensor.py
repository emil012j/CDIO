from time import sleep
from ev3dev2.sensor.lego import UltrasonicSensor

us = UltrasonicSensor()

while True: 
    distance = us.distance_centimeters
    print("Distance: {:.2f} cm".format(distance)) 
    sleep(0.5)