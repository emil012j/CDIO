# Kode til om der er forbindelse til robotten, hvis der er forbindelse til robotten så skal den kører de andre programmer. (rækkefølge?)

import socket
import json
import threading
import time

def connect_to_robot():
    while True:
        try:
            print("Connecting to robot...")
            # Create a new socket for each connection attempt
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect(("192.168.149.158", 1232))  # EV3's IP and port
            print("Connected to robot!")
            
            while True:  # Keep the connection alive and send pings
                try:
                    # Send a JSON-encoded ping message
                    ping_message = json.dumps({'ping': True}).encode()
                    client_socket.send(ping_message)
                    print("Ping sent.")
                    
                    # Optionally, receive and print the response
                    response = client_socket.recv(1024)
                    if response:
                        ack = json.loads(response.decode())
                        print("Received response:", ack)
                    time.sleep(20)  # Wait 20 seconds before the next ping
                except Exception as e:
                    print(f"Lost connection to robot: {e}")
                    break  # Exit the inner loop to reconnect
        except ConnectionRefusedError:
            print("Failed to connect to robot. Retrying in 5 seconds...")
        except Exception as e:
            print(f"Unexpected error while connecting to robot: {e}")
        time.sleep(5)  # Wait before retrying the connection

# Start the connection thread so it runs in the background
robot_connection_thread = threading.Thread(target=connect_to_robot, daemon=True)
robot_connection_thread.start()

# Keep the main thread alive
while True:
    time.sleep(1)
