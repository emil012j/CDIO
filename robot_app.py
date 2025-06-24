# -*- coding: utf-8 -*-
"""
The robot that receives commands from the vision system and executes them
Starts robot controller, starts network server, waits for commands, runs until BACK button
"""

import sys
import os
from time import sleep

# Adds src to path to be able to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.robot.controller import RobotController
from src.communication.robot_server import RobotServer

def main():
    # Starts the robot
    try:
        # Starts robot controller
        print("Robot controller...")
        robot = RobotController()
        
        # Kør 10 sekunder frem og 1 sekund bak ved opstart
        robot.move_forward_continuous(100)  # Justér hastighed efter behov
        sleep(4)
        robot.stop_continuous_move()
        sleep(0.2)
        robot.move_backward_continuous(100)
        sleep(1)
        robot.stop_continuous_move()
        sleep(0.2)
        robot.start_blockage_monitoring()
        
        # Starts network server to receive commands
        print("\n2. Starting command server...")
        server = RobotServer(robot)
        if not server.start_server():
            print("ERROR: Could not start command server")
            return
        
        print("\n3. Robot ready and waiting for commands...")
        print("Press BACK button on EV3 to exit")  # Waits for commands from the vision system
        
        # Runs until BACK button is pressed
        try:
            while robot.running:
                # Checks for button press
                if robot.check_buttons():
                    break
                
                # FASTER: Reduced delay for faster response (from 0.1 to 0.02)
                sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
            
    except Exception as e:
        print("Error in robot: {}".format(e))
        
    finally:
        print("\nShutting down robot...")
        try:
            server.stop_server()
            robot.cleanup()
        except:
            pass
        print("Robot is shut down")

if __name__ == "__main__":
    main() 