# -*- coding: utf-8 -*-
"""
Netværk server på port 1233, modtager/sender kommandoer til vision system"""

import socket
import json
import threading
from time import sleep
from ..config.settings import *

#netværk server på port 1233, modtager/sender kommandoer til vision system
class RobotServer:
    #initialiserer robotens IP og port
    def __init__(self, robot_controller, port=COMMAND_PORT):
        self.robot_controller = robot_controller
        self.port = port
        self.command_socket = None
        self.server_running = False
        
    #starter server
    def start_server(self):
        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Netværk server der lytter på port 1233
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #sikrer at server kan starte igen
            self.command_socket.bind(('', self.port)) #binds til port 1233
            self.command_socket.listen(1) #lytter på port 1233
            self.server_running = True 
            
            print("Command server listening on port {}...".format(self.port))
            
            #starter server thread
            server_thread = threading.Thread(target=self._handle_connections, daemon=True) #daemon=True sikrer at thread ikke blokerer programmet.
            server_thread.start()
            
            return True
            
        except Exception as e:
            print("Error starting command server: {}".format(e))
            return False
    
    #håndterer indkommende forbindelser
    def _handle_connections(self):
        while self.server_running and self.robot_controller.running:
            try:
                #accepter forbindelse med timeout
                self.command_socket.settimeout(1.0)
                conn, addr = self.command_socket.accept() #accepter forbindelse
                
                print("Connection from: {}".format(addr))
                
                #håndterer kommandoen
                self._handle_command(conn)
                conn.close() #lukker forbindelsen
                
            except socket.timeout:
                #tjekker om vi skal fortsætte, feks. hvis robotten ikke er klar til at modtage kommandoer
                continue
            except Exception as e:
                if self.server_running:
                    print("Error handling connection: {}".format(e))
                sleep(0.1)
    
    #håndterer en enkelt kommando fra forbindelsen
    def _handle_command(self, conn):
        try:
            # Modtager JSON kommandoer fra vision system
            data = conn.recv(1024)
            if not data:
                return
                
            #parser kommandoer ved at konvertere til JSON og gemme i command.
            command_str = data.decode('utf-8')
            command = json.loads(command_str)
            
            print("Received command: {}".format(command))
            
            #importerer movement system for at udføre kommandoer
            from ..robot.movement import execute_movement_command
            
            #sender kommandoen til movement system
            execute_movement_command(self.robot_controller, command)
            
        except json.JSONDecodeError:
            print("Invalid JSON received")
        except Exception as e:
            print("Error handling command: {}".format(e))
    
    #stopper server
    def stop_server(self):
        print("Stopping command server...")
        self.server_running = False
        
        if self.command_socket:
            try:
                self.command_socket.close()
            except Exception:
                pass
        
        print("Command server stopped") 