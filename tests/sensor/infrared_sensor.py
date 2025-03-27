from time import sleep
from ev3dev2.sensor.lego import InfraredSensor

# Initialize the sensor

ir = InfraredSensor()

while True:
    closeness = ir.value()    #Get the closeness, 0 for very close and 100 for very far
    print("Closeness:".format(closeness) )
    sleep(0.5)

