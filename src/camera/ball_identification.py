"""
References:
-----------
1. YOLO Model Training in Colab:
   - Author: Evan Juras (2025)
   - Source: "Train_YOLO_Models.ipynb" (GitHub)
   - URL: https://github.com/EdjeElectronics/Train-and-Deploy-YOLO-Models
   - Used for: 
     * Training custom YOLO models
     * Dataset preparation (Label Studio integration)
     * Model deployment examples

2. Ultralytics YOLO Documentation:
   - Source: Ultralytics Official Docs
   - URL: https://docs.ultralytics.com/
   - Used for:
     * YOLO model configuration (data.yaml)
     * Inference and training commands

3. OpenCV Python Reference:
   - Source: OpenCV 4.x Documentation
   - URL: https://docs.opencv.org/4.x/
   - Used for:
     * Video capture (cv2.VideoCapture)
     * Image processing (blur, edge detection)
     * Coordinate system visualization
     
4. Data Labeling Tool:
   - Tool: Label Studio
   - URL: https://labelstud.io/
   - Used for: Image annotation for training dataset
"""

import cv2
import numpy as np
from ultralytics import YOLO
import time

MODEL_PATH = "my_model2.pt"
SOURCE = 1  # Kamera
RESOLUTION = (1280, 720)
CONF_THRESH = 0.50

# Objektstørrelser i mm (kun relevante objekter)
BOLD_DIAMETER_MM = 40
EGG_WIDTH_MM = 50
EGG_HEIGHT_MM = 90
CROSS_DIAMETER_MM = 200

# Farver for klasser
CLASS_COLORS = {
    "cross": (0, 255, 0),
    "egg": (255, 0, 0),
    "orange ball": (0, 140, 255),
    "white ball": (255, 255, 255)
}

# Objektstørrelser i mm (kun relevante)
OBJECT_SIZES_MM = {
    "white ball": BOLD_DIAMETER_MM,
    "orange ball": BOLD_DIAMETER_MM,
    "cross": CROSS_DIAMETER_MM,
    "egg": EGG_HEIGHT_MM
}

def calculate_scale_factor(results, model):
    """ Finder skalaen baseret på 'cross' objektets størrelse. """
    for box in results.boxes:
        if model.names[int(box.cls)] == "cross":
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cross_size = max(x2 - x1, y2 - y1)
            return CROSS_DIAMETER_MM / cross_size
    return None  # Hvis ingen 'cross' findes, returnér None

def classify_egg(box, scale):
    """ Klassificerer et objekt som et æg, selv hvis det er roteret lidt. """
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    width_px = x2 - x1
    height_px = y2 - y1
    
    # Konverterer til mm
    width_mm = width_px * scale
    height_mm = height_px * scale

    # Gør det mere robust ved at justere tolerance
    dimension_match = (
        (abs(width_mm - EGG_WIDTH_MM) < 20 and abs(height_mm - EGG_HEIGHT_MM) < 20) or
        (abs(height_mm - EGG_WIDTH_MM) < 20 and abs(width_mm - EGG_HEIGHT_MM) < 20)
    )

    # Areal-baseret kontrol
    bold_area = np.pi * (BOLD_DIAMETER_MM/2)**2
    detected_area = width_mm * height_mm

    return dimension_match or (detected_area > 2.5 * bold_area)  # Mindre krav til areal

def main():
    cap = cv2.VideoCapture(SOURCE, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)

    model = YOLO(MODEL_PATH)

    last_print_time = time.time()  # Tidspunkt for sidste print
    print_interval = 5  # Print hvert 5. sekund

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Fejl: Kunne ikke læse kamera!")
                break
            
            results = model(frame, imgsz=640, conf=CONF_THRESH, verbose=False)[0]

            # Tegn koordinatsystem
            cv2.line(frame, (RESOLUTION[0] // 2, 0), (RESOLUTION[0] // 2, RESOLUTION[1]), (0, 255, 0), 2)  # Y-akse
            cv2.line(frame, (0, RESOLUTION[1] // 2), (RESOLUTION[0], RESOLUTION[1] // 2), (0, 255, 0), 2)  # X-akse

            # Beregn skalafaktor
            scale = calculate_scale_factor(results, model)

            # Print koordinater hvert 5. sekund
            if time.time() - last_print_time > print_interval:
                print("Objekter koordinater:")
                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    original_label = model.names[int(box.cls)]
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    rel_x = cx - (RESOLUTION[0] // 2)
                    rel_y = (RESOLUTION[1] // 2) - cy  # Y-aksen er vendt
                    
                    display_label = original_label  # Standardværdi
                    
                    # Klassificér kun hvis skala er tilgængelig og objekt er hvid bold
                    if original_label == "white ball" and scale is not None:
                        if classify_egg(box, scale):
                            display_label = "egg"
                    
                    print(f"Label: {display_label}, X: {rel_x}, Y: {rel_y}")
                
                last_print_time = time.time()  # Opdater tid for næste print

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                original_label = model.names[int(box.cls)]
                conf = box.conf[0].item()
                
                display_label = original_label
                
                # Klassificér æg kun hvis skala er tilgængelig
                if original_label == "white ball" and scale is not None:
                    if classify_egg(box, scale):
                        display_label = "egg"
                
                if display_label not in OBJECT_SIZES_MM:
                    continue  

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                rel_x = cx - RESOLUTION[0] // 2
                rel_y = RESOLUTION[1] // 2 - cy

                color = CLASS_COLORS.get(display_label, (255, 255, 255))
                object_size_px = max(x2 - x1, y2 - y1)

                real_size_mm = object_size_px * scale if scale else OBJECT_SIZES_MM[display_label]

                # Tegn bounding box og info
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                cv2.circle(frame, (cx, cy), 8, color, -1)

                cv2.putText(frame, f"{display_label.upper()}: {conf:.2f}", (x1, y1 - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                cv2.putText(frame, f"[X: {rel_x}, Y: {rel_y}]", (x1, y1 + 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, f"Size: {real_size_mm:.1f} mm", (x1, y1 + 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("YOLO Object Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Afslutter...")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()