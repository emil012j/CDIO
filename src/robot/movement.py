# -*- coding: utf-8 -*-
"""
tidbaseret drejning/fremad, udfører kommandoer den får fra camera
"""

from time import sleep
from ..config.settings import *

#drejning uden gyro, dvs. at robotten drejer i en bestemt vinkel i en bestemt tid
def simple_turn(robot_controller, direction, duration):
    speed = ROBOT_TURN_SPEED
    print("Simple turn: {} for {:.2f} seconds".format(direction, duration))  # Tidsbaseret drejning (uden gyro)
    
    if direction == "right":
        # Turn right: left motor forward, right motor backward (motorer er omvendt)
        robot_controller.tank_drive.on(speed, -speed)
    else:
        # Turn left: left motor backward, right motor forward (motorer er omvendt)  
        robot_controller.tank_drive.on(-speed, speed)
        
    sleep(duration)
    robot_controller.tank_drive.off()
    print("Simple turn complete")

    #fremad kørsel uden gyro, dvs. at robotten kører i en bestemt afstand i en bestemt tid, hvor den ikke tager højde for at robotten evt kører skråt
def simple_forward(robot_controller, distance_cm):
    speed = ROBOT_FORWARD_SPEED
    # Estimerer tid baseret på afstand (skal kalibreres)
    duration = distance_cm / ESTIMATED_FORWARD_RATE
    
    print("Simple forward: {:.1f} cm for {:.2f} seconds".format(distance_cm, duration))  # Tidsbaseret fremad kørsel
    
    # Begge motorer fremad (motorer er omvendt, så bruger negative værdier)
    robot_controller.tank_drive.on(-speed, -speed)
    sleep(duration)
    robot_controller.tank_drive.off()
    print("Simple forward complete")

#udfører kommandoer fra vision system
def execute_movement_command(robot_controller, command):
    try:
        cmd_type = command.get('command')  # Udfører kommandoer fra vision system
        
        if cmd_type == "simple_turn":
            direction = command.get('direction', 'left')
            duration = command.get('duration', 0.5)
            simple_turn(robot_controller, direction, duration)
            
        elif cmd_type == "simple_forward":
            distance = command.get('distance', 10)
            simple_forward(robot_controller, distance)
            
        elif cmd_type == "stop":
            robot_controller.stop_all_motors()
            print("Stop command executed")
            
        else:
            print("Unknown command: {}".format(cmd_type))
            
    except Exception as e:
        print("Error executing movement command: {}".format(e))
        robot_controller.stop_all_motors() 