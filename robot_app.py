# -*- coding: utf-8 -*-
"""
Robotten, der modtager kommandoer fra vision system, og udfører dem
 Starter robot controller+gyro, starter netværk server, venter på kommandoer, kører indtil BACK knap
"""

import sys
import os
from time import sleep

#tilføjer src til path for at kunne importere modulerne
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.robot.controller import RobotController
from src.communication.robot_server import RobotServer

def main():
    #starter robotten
    print("=" * 50)
    print("EV3 ROBOT NAVIGATION SYSTEM")
    print("=" * 50)
    
    try:
        #starter robot controller og gyro kalibrering
        print("\n1. Initializing robot controller...")
        robot = RobotController()
        
        #kalibrerer gyroen
        print("\n2. Calibrating gyro sensor...")
        robot.calibrate_gyro()
        
        #starter netværk server til at modtage kommandoer
        print("\n3. Starting command server...")
        server = RobotServer(robot)
        if not server.start_server():
            print("ERROR: Could not start command server")
            return
        
        print("\n4. Robot ready and waiting for commands...")
        print("Press BACK button on EV3 to exit")  #venter på kommandoer fra vision systemet
        
        #kører indtil BACK knap trykkes
        try:
            while robot.running:
                #tjekker for knap tryk
                if robot.check_buttons():
                    break
                
                #små delay for at undgå at CPU'en bliver for belastet
                sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
            
    except Exception as e:
        print("Fejl i robotten: {}".format(e))
        
    finally:
        print("\nSlutter robotten...")
        try:
            server.stop_server()
            robot.cleanup()
        except:
            pass
        print("Robotten er slukket ned")

if __name__ == "__main__":
    main() 