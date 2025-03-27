from ev3dev2.sensor.lego import InfraredSensor
from time import sleep

ir = InfraredSensor()  # Initialize the IR sensor

print("Starting IR sensor reading... (Press CTRL+C to stop)")
while True:
    proximity_percent = ir.proximity # property: percentage of distance (0-100)
    # Convert proximity to an approximate distance in cm, and beacon distance to cm if available
    approx_cm = proximity_percent * 0.7
    print("Proximity: {}% (~{:.1f} cm)".format(
            proximity_percent, approx_cm))
    sleep(0.5) 