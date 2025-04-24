import cv2
import numpy as np
from ultralytics import YOLO
from navigation_controller import calculate_movement
from navigation_controller import LargeMotor, OUTPUT_A, OUTPUT_B  
from navigation_controller import moveRobot  


motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)

# Kamera setup (fast monteret over banen)
cap = cv2.VideoCapture(0)
model = YOLO("camera/my_model2.pt")

def find_positions(frame):
    """Finder robot og bold-positioner i billedet"""
    results = model(frame)[0]
    robot_head, robot_tail, balls = None, None, []
    
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center = ((x1+x2)//2, (y1+y2)//2)
        
        if model.names[int(box.cls)] == "robot":
            robot_tail = center
        elif "ball" in model.names[int(box.cls)]:
            balls.append(center)
            
    return robot_tail,robot_head, balls

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # Find positioner
    robot_tail, balls = find_positions(frame)
    if not robot_tail or not balls:
        print("Ingen robot eller bolde fundet!")
        continue
    
    # Vælg nærmeste bold
    target = min(balls, key=lambda t: np.linalg.norm(np.array(t)-np.array(robot_tail)))
    
    # Brug calculate_movement funktion
    direction = calculate_movement(robot_tail, target)
    
    # Kør robotten
    moveRobot(direction)
    
    # Visualisering (til debugging) her ser vi robotten og den bold den har valgt
    # hvilken retning den skal bevæge sig i
    cv2.circle(frame, robot_tail, 10, (0,255,0), -1)  # Robotten (grøn)
    cv2.circle(frame, target, 10, (0,0,255), -1)     # Den bold robotten har valgt (rød)
    cv2.putText(frame, f"Retning: {direction}", (10,30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow("Tracking", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()