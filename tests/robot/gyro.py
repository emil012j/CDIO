import socket
import json
from threading import Thread
from time import sleep
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1

# Initialisering
tank = MoveTank(OUTPUT_A, OUTPUT_D)
gyro = GyroSensor(INPUT_1)

PORT = 12345  # Vælg en port

def turn_90_degrees(speed=20):
    gyro.reset()
    target_angle = 90
    tank.on(SpeedPercent(speed), SpeedPercent(-speed))
    while gyro.angle < target_angle:
        sleep(0.01)
    tank.off()
    print(f"Drejet {gyro.angle} grader")

def drive_forward_full_speed():
    tank.on(SpeedPercent(100), SpeedPercent(100))
    print("Kører ligeud med fuld fart")

def stop_motors():
    tank.off()
    print("Motorer stoppet")

def handle_client(conn, addr):
    print(f"Forbindelse fra {addr}")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break

            try:
                msg = json.loads(data.decode())
            except json.JSONDecodeError:
                print("Modtog ikke gyldig JSON")
                continue

            command = msg.get("command")
            if command == "start":
                print("Start kommando modtaget")
                turn_90_degrees()
                drive_forward_full_speed()
            elif command == "stop":
                print("Stop kommando modtaget")
                stop_motors()
            else:
                print(f"Ukendt kommando: {command}")

    except Exception as e:
        print(f"Fejl i klient håndtering: {e}")
    finally:
        conn.close()
        print(f"Forbindelse til {addr} lukket")

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("", PORT))
    s.listen(1)
    print(f"Server lytter på port {PORT}")

    while True:
        conn, addr = s.accept()
        client_thread = Thread(target=handle_client, args=(conn, addr), daemon=True)
        client_thread.start()

if __name__ == "__main__":
    main()
