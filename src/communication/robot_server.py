# -*- coding: utf-8 -*-
"""
Network server on port 1233, receives/sends commands to vision system"""

import socket
import json
import threading
from time import sleep
from ..config.settings import *

# Network server on port 1233, receives/sends commands to vision system
class RobotServer:
    # Initializes robot IP and port
    def __init__(self, robot_controller, port=COMMAND_PORT):
        self.robot_controller = robot_controller
        self.port = port
        self.command_socket = None
        self.server_running = False
        
    # Starts server
    def start_server(self):
        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Network server that listens on port 1233
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Ensures server can start again
            self.command_socket.bind(('', self.port)) # Binds to port 1233
            self.command_socket.listen(1) # Listens on port 1233
            self.server_running = True 
            
            print("Command server listening on port {}...".format(self.port))
            
            # Starts server thread
            server_thread = threading.Thread(target=self._handle_connections, daemon=True) # daemon=True ensures thread doesn't block the program.
            server_thread.start()
            
            return True
            
        except Exception as e:
            print("Error starting command server: {}".format(e))
            return False
    
    # Handles incoming connections
    def _handle_connections(self):
        while self.server_running and self.robot_controller.running:
            try:
                # Accept connection with timeout
                self.command_socket.settimeout(1.0)
                conn, addr = self.command_socket.accept() # Accept connection
                
                print("Connection from: {}".format(addr))
                
                # Handles the command
                self._handle_command(conn)
                conn.close() # Closes the connection
                
            except socket.timeout:
                # Checks if we should continue, e.g. if the robot is not ready to receive commands
                continue
            except Exception as e:
                if self.server_running:
                    print("Error handling connection: {}".format(e))
                sleep(0.02)  # FASTER: Reduced from 0.1 to 0.02
    
    # Handles a single command from the connection
    def _handle_command(self, conn):
        try:
            # Receives JSON commands from vision system
            data = conn.recv(1024)
            if not data:
                return
                
            # Parses commands by converting to JSON and storing in command.
            command_str = data.decode('utf-8')
            command = json.loads(command_str)
            
            print("Received command: {}".format(command))
            
            # Imports movement system to execute commands
            from ..robot.movement import execute_movement_command, execute_coordinate_command
            
            # Handles commands and coordinates
            if 'coordinates' in command:
                execute_coordinate_command(self.robot_controller, command)
            else:
                execute_movement_command(self.robot_controller, command)
            
        except json.JSONDecodeError:
            print("Invalid JSON received")
        except Exception as e:
            print("Error handling command: {}".format(e))
    
    # Stops server
    def stop_server(self):
        print("Stopping command server...")
        self.server_running = False
        
        if self.command_socket:
            try:
                self.command_socket.close()
            except Exception:
                pass
        
        print("Command server stopped") 