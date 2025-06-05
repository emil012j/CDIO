#!/usr/bin/env python3
from ev3dev2.motor  import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
import time

# ---------- Hardware ----------
gyro  = GyroSensor(INPUT_1)          # port 1
left  = LargeMotor(OUTPUT_B)         # venstre hjul
right = LargeMotor(OUTPUT_C)         # højre hjul

# ---------- Konstanter (tilpas efter dit chassis) ----------
TURN_SPEED = 20      # % motoreffekt under drej
FWD_SPEED  = 30      # % motoreffekt under fremkørsel
WHEEL_CIRC = 17.6    # hjulomkreds i cm  (mål dit eget hjul!)
TICKS_PER_CM = 360 / WHEEL_CIRC  # 360 encoder-ticks ≈ 1 omdr.

# ---------- Funktioner ----------
def gyro_turn(angle_deg):
    """
    Drej præcist 'angle_deg' grader.
    +  = med uret,  -  = mod uret.
    """
    gyro.reset()                       # nulstil til 0°
    direction = 1 if angle_deg > 0 else -1
    left.run_forever( speed_sp = -TURN_SPEED * direction)
    right.run_forever(speed_sp =  TURN_SPEED * direction)

    # Drej indtil sensoren siger ~90°
    while abs(gyro.angle) < abs(angle_deg) - 1:   # 1° dead-band
        time.sleep(0.01)

    left.stop(); right.stop()
    print(f"[EV3] Drejet {gyro.angle:.1f}°")

def drive_forward(cm):
    """
    Kør 'cm' centimeter ligeud (enkelt, encoder-baseret).
    """
    ticks = int(cm * TICKS_PER_CM)
    left.on_for_degrees( SpeedPercent(FWD_SPEED),  ticks, block=False)
    right.on_for_degrees(SpeedPercent(FWD_SPEED),  ticks)

# ---------- Hovedprogram ----------
if __name__ == "__main__":
    print("[EV3] Starter 90° drej …")
    gyro_turn(90)          # præcis kvart omgang

    time.sleep(0.5)        # kort pause

    print("[EV3] Kører 30 cm frem …")
    drive_forward(30)

    print("[EV3] Færdig.")
