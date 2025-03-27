from time import sleep
from ev3dev2.sensor.lego import ColorSensor 

cs = ColorSensor()

while True:
    color = cs.color 
    ambient = cs.ambient_light_intensity    # Get ambient light level from 0-100 
    reflected = cs.reflected_light_intensity    # Get reflected light level from 0-100

    print(f"Color: {color}, Ambient Light: {ambient}, Reflected Light: {reflected}")
    sleep(0.5)

    # The light level can be used to determine light colors, i.e. the ball