import cv2
from ultralytics import YOLO

# Indstillinger
MODEL_PATH = "yolov8n.pt"
SOURCE = 1  # Prøv 0, 1, 2 hvis kamera ikke virker
RESOLUTION = (1280, 720)
CONF_THRESH = 0.6

def draw_coordinate_system(frame):
    h, w = frame.shape[:2]
    # Tegn X- og Y-akser (tykkere linjer)
    cv2.line(frame, (w//2, 0), (w//2, h), (200, 200, 200), 2)  # Y-akse (tynd)
    cv2.line(frame, (0, h//2), (w, h//2), (200, 200, 200), 2)  # X-akse (tynd)
    # Tilføj retningsmarkører (større tekst)
    cv2.putText(frame, '+X', (w-50, h//2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
    cv2.putText(frame, '-X', (10, h//2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
    cv2.putText(frame, '+Y', (w//2 + 20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
    cv2.putText(frame, '-Y', (w//2 + 20, h-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)

def setup_camera():
    cap = cv2.VideoCapture(SOURCE, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manuel eksponering
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)  # Justér lys (-7 = mørkere, +4 = lysere) -7 for webcam
    return cap

def main():
    cap = setup_camera()
    model = YOLO(MODEL_PATH)
    
    # Farver for hver klasse (BGR)
    colors = [
        (0, 165, 255),  # Orange (tydelig)
        (0, 255, 0),    # Grøn
        (255, 0, 0)     # Rød
    ]
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Fejl: Kunne ikke læse kamera!")
                break
            
            # Kør YOLO-detektion
            results = model(frame, imgsz=640, conf=CONF_THRESH, verbose=False)[0]
            
            # Tegn koordinatsystem
            draw_coordinate_system(frame)
            
            # Behandl hver detektion
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls)]
                conf = box.conf[0].item()
                
                # Beregn center og relative koordinater
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                rel_x = cx - RESOLUTION[0] // 2
                rel_y = RESOLUTION[1] // 2 - cy  # Vend Y-aksen
                
                # Vælg farve baseret på klasse
                color = colors[int(box.cls) % len(colors)]
                
                # Tegn bounding box (tykkere linje)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                
                # Tegn centerpunkt (større cirkel)
                cv2.circle(frame, (cx, cy), 8, color, -1)
                
                # Vis label og confidence (større tekst)
                cv2.putText(
                    frame, 
                    f"{label.upper()}: {conf:.2f}", 
                    (x1, y1 - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.9, 
                    color, 
                    2
                )
                
                # Vis koordinater (under label)
                cv2.putText(
                    frame, 
                    f"[X: {rel_x}, Y: {rel_y}]", 
                    (x1, y1 + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, 
                    (255, 255, 255),  # Hvid tekst for bedre læsbarhed
                    2
                )
            
            cv2.imshow("Object Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()