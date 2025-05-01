"""
Network communication module for the EV3 robot
Handles socket connections and data transfer
"""

import socket
import json
import threading
import time

class NetworkClient:
    """
    TCP client for communicating with the camera server
    
    Handles connection establishment, data reception, and message parsing
    """
    
    def __init__(self, server_ip, server_port=5000, reconnect_attempts=5):
        """Initialize the network client"""
        self.server_ip = server_ip
        self.server_port = server_port
        self.reconnect_attempts = reconnect_attempts
        self.socket = None
        self.connected = False
        self.running = False
        
        # Buffer for incoming data
        self.receive_buffer = ""
        
        # Latest received data
        self.latest_data = None
        self.data_lock = threading.Lock()
        
        # Callback function for data processing
        self.data_callback = None
    
    def connect(self):
        """Connect to the server"""
        if self.connected:
            return True
            
        attempts = 0
        while attempts < self.reconnect_attempts:
            try:
                print(f"Connecting to {self.server_ip}:{self.server_port} (attempt {attempts+1}/{self.reconnect_attempts})...")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5)  # 5 second timeout for connection
                self.socket.connect((self.server_ip, self.server_port))
                self.socket.settimeout(None)  # Reset timeout for normal operation
                self.connected = True
                print("Connected successfully!")
                return True
                
            except Exception as e:
                print(f"Connection failed: {e}")
                attempts += 1
                if self.socket:
                    self.socket.close()
                    self.socket = None
                time.sleep(2)
                
        print("All connection attempts failed")
        return False
    
    def disconnect(self):
        """Disconnect from the server"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        self.connected = False
    
    def set_data_callback(self, callback):
        """Set callback function to be called when new data is received"""
        self.data_callback = callback
    
    def start_receiver(self):
        """Start the data receiver thread"""
        if not self.connected:
            print("Cannot start receiver: not connected")
            return False
            
        self.running = True
        receiver_thread = threading.Thread(target=self._receive_loop)
        receiver_thread.daemon = True
        receiver_thread.start()
        return True
    
    def _receive_loop(self):
        """Background thread for receiving data"""
        while self.running and self.connected:
            try:
                # Receive data
                data = self.socket.recv(4096)
                
                if not data:
                    # Connection closed
                    print("Server closed connection")
                    self.connected = False
                    break
                
                # Add to buffer and process
                self.receive_buffer += data.decode('utf-8')
                self._process_buffer()
                
            except Exception as e:
                print(f"Error receiving data: {e}")
                self.connected = False
                break
    
    def _process_buffer(self):
        """Process received data buffer to extract complete messages"""
        while '\n' in self.receive_buffer:
            # Split at newline
            line, self.receive_buffer = self.receive_buffer.split('\n', 1)
            
            try:
                # Parse JSON data
                data = json.loads(line)
                
                # Update latest data
                with self.data_lock:
                    self.latest_data = data
                
                # Call callback if set
                if self.data_callback:
                    self.data_callback(data)
                    
            except json.JSONDecodeError as e:
                print(f"Error parsing JSON: {e}")
    
    def get_latest_data(self):
        """Get the latest received data"""
        with self.data_lock:
            return self.latest_data
    
    def send_data(self, data):
        """Send data to the server"""
        if not self.connected or not self.socket:
            return False
            
        try:
            # Convert to JSON and add newline
            json_data = json.dumps(data) + '\n'
            self.socket.sendall(json_data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending data: {e}")
            self.connected = False
            return False 