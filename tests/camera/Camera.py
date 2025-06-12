import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from ultralytics import YOLO
from src.camera.detection import load_yolo_model, run_detection, process_detections_and_draw
from src.config.settings import CAMERA_SOURCE

def main():
    # Load YOLO model
    model = load_yolo_model()
    if model is None:
        print("Could not load YOLO model.")
        return

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open camera.")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame.")
            break

        # Run detection
        results = run_detection(model, frame)
        display_frame = frame.copy()
        # Draw detections (no scale factor needed for just visualization)
        process_detections_and_draw(results, model, display_frame)

        cv2.imshow("YOLO Live Detection", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
