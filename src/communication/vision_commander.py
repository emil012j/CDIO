# -*- coding: utf-8 -*-
"""
Håndterer netværk forbindelse(TCP socket), sender JSON kommandoer(feks. drejning, fremad, stop som JSON), 
rate limiting (ikke for mange kommandoer per sekund, da robotten ikke kan håndtere dem), 
timeout/error handling (hvis robotten ikke svarer)
"""

import socket
import json
import time
from ..config.settings import *

#håndterer netværk forbindelse, sender JSON kommandoer, rate limiting, timeout/error handling
class VisionCommander:
    #initialiserer robotens IP og port
    def __init__(self, robot_ip=ROBOT_IP, command_port=COMMAND_PORT):
        self.robot_ip = robot_ip
        self.command_port = command_port
        self.last_command_time = 0
        
    #tjekker om der er gået en bestemt tid siden sidste kommando
    def can_send_command(self):
        current_time = time.time()
        return current_time - self.last_command_time >= COMMAND_COOLDOWN  # Rate limiting (ikke for mange kommandoer per sekund)
    
    #sender JSON kommandoer over TCP socket
    def send_command(self, command_dict):
        if not self.can_send_command():
            return False
            
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Håndterer netværk forbindelse til EV3
            sock.settimeout(2.0)  # Timeout og error handling
            sock.connect((self.robot_ip, self.command_port))
            
            # Sender JSON kommandoer over TCP socket
            command_json = json.dumps(command_dict).encode()
            sock.send(command_json)
            sock.close()
            
            self.last_command_time = time.time()
            return True
            
        except socket.timeout:
            print("Timeout connecting to robot")
            return False
        except ConnectionRefusedError:
            print("Robot refused connection - check if robot is running")
            return False
        except Exception as e:
            print("Error sending command: {}".format(e))
            return False
    
    #sender en drejning kommando ved at sende en JSON med kommandoen, retningen og tiden. feks. {"command": "simple_turn", "direction": "left", "duration": 1.0}
    def send_turn_command(self, direction, duration):
        """Send a turn command to the robot"""
        command = {
            "command": "simple_turn",
            "direction": direction,
            "duration": duration
        }
        return self.send_command(command)
    
    #sender en fremad kommando ved at sende en JSON med kommandoen og afstanden. feks. {"command": "forward", "distance": 10}
    def send_forward_command(self, distance):
        """Send a forward movement command to the robot"""
        command = {
            "command": "forward", 
            "distance": distance
        }
        return self.send_command(command)
    
    #sender en stop kommando ved at sende en JSON med kommandoen. feks. {"command": "stop"}
    def send_stop_command(self):
        """Send a stop command to the robot"""
        command = {
            "command": "stop"
        }
        return self.send_command(command) 