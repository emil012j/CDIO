# Kode til om kameraet virker og opfanger banen, boldene, væggene og koordinater?

import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import json
import threading
import math

class CameraProcessor:
    def __init__(self, model_path="src/camera/best.pt", camera_id=1, resolution=(1280, 720), 
                 host='0.0.0.0', port=5000, confidence=0.35):
        self.model_path = model_path
        self.camera_id = camera_id
        self.resolution = resolution
        self.confidence = confidence
        self.host = host
        self.port = port
        
        # Server for communication with robot
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        self.running = False
        
        # Class colors for visualization
        self.class_colors = {
            "cross": (0, 255, 0), 
            "egg": (255, 0, 0), 
            "orange ball": (0, 140, 255), 
            "white ball": (255, 255, 255),
            "robothead": (0, 0, 255), 
            "robottail": (255, 0, 255)
        }
        
        # Constants from ball_identification.py
        self.EGG_SIZE_THRESHOLD_MM = 58.0
        self.BOLD_DIAMETER_MM = 40
        self.CROSS_DIAMETER_MM = 200.0
        self.EGG_OBB_ASPECT_RATIO_THRESHOLD = 1.3
        self.ORIENTED_OBJECTS = ["robothead", "robottail", "egg", "cross"]
    
    def initialize(self):
        """Initialize the camera and model"""
        print("Initializing camera and model...")
        
        # Load YOLO model
        try:
            self.model = YOLO(self.model_path)
            print(f"Model loaded: {self.model_path}")
        except Exception as e:
            print(f"Error loading model: {e}")
            return False
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_id)
            
        if not self.cap.isOpened():
            print("Failed to open camera")
            return False
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)
        
        print("Camera initialized successfully")
        return True
    
    def start_server(self):
        """Start TCP server to communicate with robot"""
        server_thread = threading.Thread(target=self._run_server)
        server_thread.daemon = True
        server_thread.start()
        
    def _run_server(self):
        """Run TCP server in a separate thread"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        print(f"Server started on {self.host}:{self.port}")
        
        while self.running:
            try:
                print("Waiting for robot connection...")
                self.client_socket, self.client_addr = self.server_socket.accept()
                print(f"Robot connected from {self.client_addr}")
                
                # Keep connection alive until disconnect
                while self.running:
                    # Just keep the connection open
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Server error: {e}")
                time.sleep(1)
                
            finally:
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
    
    def send_data_to_robot(self, data):
        """Send detection data to robot"""
        if not self.client_socket:
            return False
            
        try:
            # Convert data dictionary to JSON string
            data_json = json.dumps(data)
            # Send data with a newline as message delimiter
            self.client_socket.sendall(f"{data_json}\n".encode('utf-8'))
            return True
        except Exception as e:
            print(f"Error sending data to robot: {e}")
            self.client_socket = None
            return False
    
    def get_obb_dimensions(self, obb_results, index):
        """Extract width and height from OBB results similar to ball_identification.py"""
        try:
            if hasattr(obb_results, 'wh') and obb_results.wh is not None and len(obb_results.wh) > index:
                wh = obb_results.wh[index].cpu().numpy()
                w, h = min(wh), max(wh)
                return float(w), float(h)
            elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
                points = obb_results.xyxyxyxy[index].cpu().numpy().reshape(4, 2)
                side1 = np.linalg.norm(points[0]-points[1])
                side2 = np.linalg.norm(points[1]-points[2])
                w, h = min(side1, side2), max(side1, side2)
                return float(w), float(h)
            elif hasattr(obb_results, 'xyxy') and obb_results.xyxy is not None and len(obb_results.xyxy) > index:
                x1, y1, x2, y2 = map(int, obb_results.xyxy[index])
                return float(x2-x1), float(y2-y1)
            else:
                return None, None
        except Exception as e:
            print(f"Error getting OBB dimensions: {e}")
            return None, None
    
    def is_likely_egg_by_shape(self, obb_results, index):
        """Check if an object is likely an egg based on its shape"""
        try:
            obb_w, obb_h = self.get_obb_dimensions(obb_results, index)
            if obb_w is not None and obb_h is not None and obb_w > 1e-3:
                aspect_ratio = obb_h / obb_w
                return aspect_ratio >= self.EGG_OBB_ASPECT_RATIO_THRESHOLD
            return False
        except Exception as e:
            print(f"Error checking egg shape: {e}")
            return False
    
    def calculate_orientation(self, obb_results, index, label):
        """Calculate orientation of an object"""
        try:
            if hasattr(obb_results, 'angle') and obb_results.angle is not None and len(obb_results.angle) > index:
                return float(obb_results.angle[index])
            elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
                points = obb_results.xyxyxyxy[index].cpu().numpy().reshape(4, 2)
                s1 = np.linalg.norm(points[0]-points[1])
                s2 = np.linalg.norm(points[1]-points[2])
                if s1 > s2:
                    dx = points[1][0]-points[0][0]
                    dy = points[1][1]-points[0][1]
                else:
                    dx = points[2][0]-points[1][0]
                    dy = points[2][1]-points[1][1]
                return math.degrees(math.atan2(-dy, dx))
            elif hasattr(obb_results, 'xyxy') and obb_results.xyxy is not None and len(obb_results.xyxy) > index:
                x1, y1, x2, y2 = map(int, obb_results.xyxy[index])
                w = x2-x1
                h = y2-y1
                if h > 0 and w > 0 and label in ["egg", "cross"] and (w/h > 1.2 or h/w > 1.2):
                    return 90.0 if h > w else 0.0
            return 0.0
        except Exception as e:
            print(f"Error calculating orientation: {e}")
            return 0.0
    
    def process_frame(self, frame):
        """Process a single frame to detect objects using methods from ball_identification.py"""
        # Create empty dictionaries for detections
        objects = {
            "robot": {"head": None, "tail": None, "center": None, "orientation": None},
            "targets": [],
            "obstacles": []
        }
        
        # Reference coordinates (center of the image)
        center_x, center_y = self.resolution[0] // 2, self.resolution[1] // 2
        
        # Draw grid lines
        cv2.line(frame, (center_x, 0), (center_x, self.resolution[1]), (0, 255, 0), 1)
        cv2.line(frame, (0, center_y), (self.resolution[0], center_y), (0, 255, 0), 1)
        
        # Run detection model
        try:
            results = self.model(frame, imgsz=640, conf=self.confidence, verbose=False)
            if results and len(results) > 0:
                result = results[0]
                
                # First check for OBB results (like in ball_identification.py)
                if hasattr(result, 'obb') and result.obb is not None and hasattr(result.obb, 'cls') and result.obb.cls is not None:
                    # Process OBB detections
                    for i in range(len(result.obb.cls)):
                        try:
                            cls_id = int(result.obb.cls[i])
                            conf = float(result.obb.conf[i]) if hasattr(result.obb, 'conf') else 0.0
                            label = self.model.names[cls_id]
                            
                            # Get position information
                            cx, cy = None, None
                            obb_points = None
                            
                            if hasattr(result.obb, 'xyxyxyxy') and result.obb.xyxyxyxy is not None and len(result.obb.xyxyxyxy) > i:
                                obb_points = result.obb.xyxyxyxy[i].cpu().numpy().reshape(4, 2).astype(np.int32)
                                x_coords = obb_points[:, 0]
                                y_coords = obb_points[:, 1]
                                cx = int(np.mean(x_coords))
                                cy = int(np.mean(y_coords))
                            elif hasattr(result.obb, 'xywh') and result.obb.xywh is not None and len(result.obb.xywh) > i:
                                cx, cy, _, _ = result.obb.xywh[i]
                                cx, cy = int(cx), int(cy)
                            elif hasattr(result.obb, 'xyxy') and result.obb.xyxy is not None and len(result.obb.xyxy) > i:
                                x1, y1, x2, y2 = map(int, result.obb.xyxy[i])
                                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                            
                            if cx is None or cy is None:
                                continue
                                
                            # Check if white ball is actually an egg
                            if label == "white ball" and self.is_likely_egg_by_shape(result.obb, i):
                                label = "egg"
                            
                            # Calculate relative coordinates
                            rel_x = cx - center_x
                            rel_y = center_y - cy  # Invert Y to match standard coordinate system
                            
                            # Process based on object type
                            if label == "robothead":
                                objects["robot"]["head"] = (rel_x, rel_y)
                            elif label == "robottail":
                                objects["robot"]["tail"] = (rel_x, rel_y)
                            elif label in ["white ball", "orange ball"]:
                                objects["targets"].append({
                                    "type": label,
                                    "position": (rel_x, rel_y),
                                    "confidence": conf
                                })
                            elif label == "egg":
                                objects["obstacles"].append({
                                    "type": label,
                                    "position": (rel_x, rel_y),
                                    "confidence": conf
                                })
                            
                            # Draw detection on the frame
                            color = self.class_colors.get(label, (128, 128, 128))
                            
                            # Draw either polygon or rectangle
                            if obb_points is not None:
                                cv2.polylines(frame, [obb_points], isClosed=True, color=color, thickness=2)
                            else:
                                # Fallback to rectangle if we have xyxy data
                                if hasattr(result.obb, 'xyxy') and result.obb.xyxy is not None and len(result.obb.xyxy) > i:
                                    x1, y1, x2, y2 = map(int, result.obb.xyxy[i])
                                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            
                            # Draw label and center point
                            cv2.putText(frame, f"{label} {conf:.2f}", (cx, cy-15), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            cv2.circle(frame, (cx, cy), 5, color, -1)
                            
                        except Exception as e:
                            print(f"Error processing OBB detection {i}: {e}")
                
                # Fallback to standard boxes if OBB not available
                elif hasattr(result, 'boxes') and result.boxes is not None:
                    for i, box in enumerate(result.boxes):
                        try:
                            cls_id = int(box.cls[0])
                            conf = float(box.conf[0])
                            label = self.model.names[cls_id]
                            
                            # Get bounding box
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                            
                            # Relative coordinates from center
                            rel_x = cx - center_x
                            rel_y = center_y - cy  # Invert Y to match standard coordinate system
                            
                            # Process based on object type
                            if label == "robothead":
                                objects["robot"]["head"] = (rel_x, rel_y)
                            elif label == "robottail":
                                objects["robot"]["tail"] = (rel_x, rel_y)
                            elif label in ["white ball", "orange ball"]:
                                objects["targets"].append({
                                    "type": label,
                                    "position": (rel_x, rel_y),
                                    "confidence": conf
                                })
                            elif label == "egg":
                                objects["obstacles"].append({
                                    "type": label,
                                    "position": (rel_x, rel_y),
                                    "confidence": conf
                                })
                                
                            # Draw detection on the frame
                            color = self.class_colors.get(label, (128, 128, 128))
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1-5), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            cv2.circle(frame, (cx, cy), 5, color, -1)
                            
                        except Exception as e:
                            print(f"Error processing box detection {i}: {e}")
                
                # Calculate robot center and orientation if both parts are detected
                if objects["robot"]["head"] and objects["robot"]["tail"]:
                    head_x, head_y = objects["robot"]["head"]
                    tail_x, tail_y = objects["robot"]["tail"]
                    
                    # Calculate center
                    center_x_robot = (head_x + tail_x) // 2
                    center_y_robot = (head_y + tail_y) // 2
                    objects["robot"]["center"] = (center_x_robot, center_y_robot)
                    
                    # Calculate orientation (angle in degrees)
                    dx = head_x - tail_x
                    dy = head_y - tail_y
                    orientation = np.degrees(np.arctan2(dy, dx))
                    objects["robot"]["orientation"] = orientation
                    
                    # Draw robot orientation line
                    cv2.line(frame, 
                           (tail_x + center_x, tail_y + center_y),
                           (head_x + center_x, head_y + center_y), 
                           (0, 255, 255), 2)
                    
                    # Draw extended line for better visualization
                    # Calculate normalized direction vector
                    length = math.sqrt(dx*dx + dy*dy)
                    if length > 1e-6:
                        norm_dx = dx / length
                        norm_dy = dy / length
                        line_length = 2000  # Long line for visualization
                        
                        # Draw line extending from head
                        ex = int(head_x + norm_dx * line_length) + center_x
                        ey = int(head_y + norm_dy * line_length) + center_y
                        sx = head_x + center_x
                        sy = head_y + center_y
                        cv2.line(frame, (sx, sy), (ex, ey), (0, 255, 255), 1)
        
        except Exception as e:
            print(f"Error in detection: {e}")
        
        return frame, objects
    
    def run(self):
        """Main loop to process video and send data"""
        if not self.initialize():
            return
            
        # Start server
        self.running = True
        self.start_server()
        
        try:
            while self.running:
                # Read frame
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                # Process frame
                display_frame, objects = self.process_frame(frame.copy())
                
                # Send data to robot if connected
                if self.client_socket and objects["robot"]["center"]:
                    self.send_data_to_robot(objects)
                
                # Display the frame
                cv2.imshow("Camera Feed", display_frame)
                
                # Check for exit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.running = False
            if self.client_socket:
                self.client_socket.close()
            if self.server_socket:
                self.server_socket.close()
            self.cap.release()
            cv2.destroyAllWindows()
            print("Camera processor stopped")

if __name__ == "__main__":
    processor = CameraProcessor()
    processor.run()