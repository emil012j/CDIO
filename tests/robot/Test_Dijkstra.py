import socket
import json
import time

ROBOT_IP = "192.168.62.158"  # ← skift IP hvis nødvendigt
COMMAND_PORT = 1233

def send_turn_command(angle_deg):
    try:
        print(f"Sender turn {angle_deg}° til robotten...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, COMMAND_PORT))
        command = {"command": "turn", "angle": angle_deg}
        sock.send(json.dumps(command).encode())
        sock.close()
        print("Kommando sendt.")
    except Exception as e:
        print(f"Fejl ved sending: {e}")

if __name__ == "__main__":
    send_turn_command(90)  # Drej 90 grader
    time.sleep(5)  # Vent lidt, så motoren når at dreje
    print("Test færdig.")
