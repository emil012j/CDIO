# -*- coding: utf-8 -*-
"""
YOLO model loading, object detection and results
"""

import cv2
import numpy as np
from ultralytics import YOLO
import math
import os
from ..config.settings import *

# Finds the width and height of the object
def get_obb_dimensions_from_obb_results(obb_results, index):
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
        else: 
            return None, None
    except Exception: 
        return None, None

# Checks if it is likely an egg by OBB shape
def is_likely_egg_by_obb_shape_idx(obb_results, index):
    try:
        obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(obb_results, index)
        if obb_w_px is not None and obb_h_px is not None and obb_w_px > 1e-3:
            aspect_ratio = obb_h_px / obb_w_px
            return aspect_ratio >= EGG_OBB_ASPECT_RATIO_THRESHOLD
        else: 
            return False
    except Exception: 
        return False

def get_class_id(class_name_lower, model):
    for idx, name in model.names.items():
        if name.lower() == class_name_lower: 
            return idx
    return None


# Calculates scaling factor based on the red cross
def calculate_scale_factor(results, model):
    try:
        if results is None or not hasattr(results, 'obb') or results.obb is None: 
            return None
        if not hasattr(results.obb, 'cls') or results.obb.cls is None: 
            return None
        num_obb_boxes = len(results.obb.cls)
        if num_obb_boxes == 0: 
            return None
        cross_id_target = get_class_id('cross', model)
        if cross_id_target is None: 
            return None
        for i in range(num_obb_boxes):
             cls_id = int(results.obb.cls[i])
             conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
             if cls_id == cross_id_target and conf >= CONFIDENCE_THRESHOLD:
                obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(results.obb, i)
                if obb_w_px is not None and obb_h_px is not None:
                    cross_obb_size_px = (obb_w_px + obb_h_px) / 2.0
                    if cross_obb_size_px > 1: 
                        return CROSS_DIAMETER_MM / cross_obb_size_px  # Calculates scale factor based on cross size
        return None
    except Exception: 
        return None

# Finds the position of the red cross in the image
def get_cross_position(results, model):
    try:
        print(f"[DEBUG] get_cross_position: Entering function. results is None: {results is None}")
        if results is None or not hasattr(results, 'obb') or results.obb is None:
            print("[DEBUG] get_cross_position: results or results.obb is None/invalid. Returning None.")
            return None
        
        cross_id_target = get_class_id('cross', model)
        print(f"[DEBUG] get_cross_position: cross_id_target = {cross_id_target}")
        if cross_id_target is None:
            print("[DEBUG] get_cross_position: cross_id_target is None. 'cross' class not found in model.names. Returning None.")
            return None
        
        print(f"[DEBUG] get_cross_position: Number of OBB detections: {len(results.obb.cls)}")
        for i in range(len(results.obb.cls)):
            cls_id = int(results.obb.cls[i])
            print(f"[DEBUG] get_cross_position: Checking detection {i}, cls_id: {cls_id}")
            if cls_id == cross_id_target:
                # Attempt to get OBB info from xywhr (oriented bounding box with rotation)
                if hasattr(results.obb, 'xywhr') and results.obb.xywhr is not None and len(results.obb.xywhr) > i:
                    cx, cy, w, h, _ = results.obb.xywhr[i].cpu().numpy()
                    angle = calculate_orientation(results.obb, i, model.names[cls_id])
                    print(f"[DEBUG] get_cross_position: Cross detected using xywhr! OBB: center=({cx}, {cy}), dim=({w}, {h}), angle={angle}")
                # Fallback to xyxy if xywhr is not available (axis-aligned bounding box)
                elif hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None and len(results.obb.xyxy) > i:
                    box = results.obb.xyxy[i].cpu().numpy()
                    cx = (box[0] + box[2]) / 2
                    cy = (box[1] + box[3]) / 2
                    w = box[2] - box[0]
                    h = box[3] - box[1]
                    angle = 0.0 # Assume 0 angle for axis-aligned box
                    print(f"[DEBUG] get_cross_position: Cross detected using xyxy! AABB: center=({cx}, {cy}), dim=({w}, {h}), angle={angle}")
                else:
                    print(f"[DEBUG] get_cross_position: Cross detected (cls_id={cls_id}), but neither xywhr nor xyxy attributes found for this detection. Skipping.")
                    continue # Skip this detection if no usable bounding box format

                return {
                    'center_x': float(cx),
                    'center_y': float(cy),
                    'width': float(w),
                    'height': float(h),
                    'angle': float(angle)
                }
        print("[DEBUG] get_cross_position: Cross not found in detections. Returning None.")
        return None
    except Exception as e:
        print(f"[DEBUG] get_cross_position: An exception occurred: {e}. Returning None.")
        return None
    
# Calculates the orientation of the object
def calculate_orientation(obb_results, index, label):
    try:
        if hasattr(obb_results, 'angle') and obb_results.angle is not None and len(obb_results.angle)>index and obb_results.angle[index] is not None: 
            return float(obb_results.angle[index])
        elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
           points = obb_results.xyxyxyxy[index].cpu().numpy().reshape(4,2)
           s1 = np.linalg.norm(points[0]-points[1])
           s2 = np.linalg.norm(points[1]-points[2])
           if s1>s2: 
               dx = points[1][0]-points[0][0]
               dy = points[1][1]-points[0][1]
           else: 
               dx = points[2][0]-points[1][0]
               dy = points[2][1]-points[1][1]
           return math.degrees(math.atan2(-dy, dx))
        elif hasattr(obb_results, 'xyxy') and obb_results.xyxy is not None and len(obb_results.xyxy) > index:
            x1,y1,x2,y2 = map(int,obb_results.xyxy[index])
            w = x2-x1
            h = y2-y1
            if h>0 and w>0 and label in ["egg","cross"] and (w/h>1.2 or h/w>1.2): 
                return 90.0 if h>w else 0.0
        return 0.0
    except Exception: 
        return 0.0

# Loads YOLO model
def load_yolo_model(model_path=MODEL_PATH):
    try:
        if not os.path.exists(model_path): 
            print("CANNOT FIND {}".format(model_path))
            return None
        print("LOADING YOLO MODEL...")
        model = YOLO(model_path)  # Loads YOLO model (best.pt)
        if get_class_id('cross', model) is None: 
            print("CROSS NOT FOUND")
        return model
    except Exception as e: 
        print("ERROR loading model: {}".format(e))
        return None
    
# DETECTS OBJECTS IN THE CAMERA
def run_detection(model, frame):
    try:
        results_list = model(frame, imgsz=640, conf=CONFIDENCE_THRESHOLD, verbose=False)  # Runs detection on camera frames
        if isinstance(results_list, list) and len(results_list) > 0:
            return results_list[0]
        return None
    except Exception: 
        return None
    
# PROCESSES AND DRAWS DETECTIONS AS IN THE OLD FILE
def process_detections_and_draw(results, model, frame, scale_factor=None):
    robot_head = None
    robot_tail = None
    balls = []
    walls = []  # Add walls to detection
    log_info_list = []
    
    # Check if we have valid detections
    if results is None or not hasattr(results, 'obb') or results.obb is None:
        return robot_head, robot_tail, balls, walls, log_info_list
    if not hasattr(results.obb, 'cls') or results.obb.cls is None:
        return robot_head, robot_tail, balls, walls, log_info_list
    
    try: 
        num_detections = len(results.obb.cls)
    except TypeError: 
        return robot_head, robot_tail, balls, walls, log_info_list
    
    if num_detections == 0:
        return robot_head, robot_tail, balls, walls, log_info_list
    
    # Draw centerlines as in the old file
    height, width = frame.shape[:2]
    cv2.line(frame, (width//2, 0), (width//2, height), (0,255,0), 1)
    cv2.line(frame, (0, height//2), (width, height//2), (0,255,0), 1)
    
    for i in range(num_detections):
        try:
            cls_id = int(results.obb.cls[i])
            conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
            if cls_id < 0 or cls_id >= len(model.names): 
                continue
            
            original_label = model.names[cls_id]
            display_label = original_label
            
            # Get position information as in the old file
            x1, y1, x2, y2, cx_calc, cy_calc = 0, 0, 0, 0, 0, 0
            has_pos_info = False
            obb_points = None
            
            if hasattr(results.obb, 'xyxyxyxy') and results.obb.xyxyxyxy is not None and len(results.obb.xyxyxyxy) > i:
                obb_points = results.obb.xyxyxyxy[i].cpu().numpy().reshape(4, 2).astype(np.int32)
                x_coords = obb_points[:,0]
                y_coords = obb_points[:,1]
                x1, y1 = np.min(x_coords), np.min(y_coords)
                x2, y2 = np.max(x_coords), np.max(y_coords)
                cx_calc = int(np.mean(x_coords))
                cy_calc = int(np.mean(y_coords))
                has_pos_info = True
            elif hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None and len(results.obb.xyxy) > i:
                x1, y1, x2, y2 = map(int, results.obb.xyxy[i])
                cx_calc, cy_calc = (x1+x2)//2, (y1+y2)//2
                has_pos_info = True
            # Prioritize xywhr for oriented bounding boxes
            elif hasattr(results.obb, 'xywhr') and results.obb.xywhr is not None and len(results.obb.xywhr) > i:
                cx_whr, cy_whr, w_whr, h_whr, _ = results.obb.xywhr[i].cpu().numpy()
                cx_calc, cy_calc = int(cx_whr), int(cy_whr)
                # For drawing, approximate x1,y1,x2,y2 from center, width, height
                x1 = int(cx_whr - w_whr / 2)
                y1 = int(cy_whr - h_whr / 2)
                x2 = int(cx_whr + w_whr / 2)
                y2 = int(cy_whr + h_whr / 2)
                has_pos_info = True
            
            if not has_pos_info:
                continue
                
            # Calculate real world dimensions as in the old file
            real_w_mm, real_h_mm, dim_method = None, None, "N/A"
            if scale_factor is not None:
                obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(results.obb, i)
                if obb_w_px is not None:
                    real_w_mm = obb_w_px * scale_factor
                    real_h_mm = obb_h_px * scale_factor
                    dim_method = "OBB"
                elif x2 > x1 and y2 > y1:
                    w_px = x2 - x1
                    h_px = y2 - y1
                    real_w_mm = w_px * scale_factor
                    real_h_mm = h_px * scale_factor
                    dim_method = "xyxy"
            
            # Egg detection as in the old file
            is_egg = False
            egg_reason = ""
            if original_label == "white ball":
                is_egg_shape = is_likely_egg_by_obb_shape_idx(results.obb, i)
                if is_egg_shape:
                    egg_reason = "Shape"
                is_egg_size = False
                if real_w_mm is not None and real_h_mm is not None and (real_w_mm > EGG_SIZE_THRESHOLD_MM or real_h_mm > EGG_SIZE_THRESHOLD_MM):
                    is_egg_size = True
                    if egg_reason:
                        egg_reason += "+Size"
                    else:
                        egg_reason = "Size"
                is_egg = is_egg_shape or is_egg_size
                if is_egg:
                    display_label = "egg"
            
            # Calculate relative coordinates from center
            rel_x = cx_calc - width // 2
            rel_y = height // 2 - cy_calc
            
            # Orientation for oriented objects
            orientation_deg = 0.0
            if display_label in ORIENTED_OBJECTS:
                orientation_deg = calculate_orientation(results.obb, i, display_label)
                if display_label == "robothead":
                    robot_head = {"pos": (cx_calc, cy_calc), "orientation": orientation_deg}
                elif display_label == "robottail":
                    robot_tail = {"pos": (cx_calc, cy_calc), "orientation": orientation_deg}
            
            # Store balls (not eggs) and walls
            if "ball" in display_label.lower() and "egg" not in display_label.lower():
                balls.append((cx_calc, cy_calc))
            elif display_label.lower() == "wall":
                walls.append((cx_calc, cy_calc))
            
            # Draw OBB or rectangle as in the old file
            color = CLASS_COLORS.get(display_label, (128, 128, 128))
            if obb_points is not None:
                cv2.polylines(frame, [obb_points], isClosed=True, color=color, thickness=2)
            elif x2 > x1:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            
            # Draw labels as in the old file
            y_offset_text = y1 + 20
            label_conf_text = "{} ({:.2f})".format(display_label, conf)
            cv2.putText(frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
            cv2.putText(frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            
            if is_egg:
                reason_text = "EGG ({})".format(egg_reason)
                cv2.putText(frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
                cv2.putText(frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            
            # Coordinate text
            coord_text = "[X:{}, Y:{}]".format(rel_x, rel_y)
            cv2.putText(frame, coord_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            y_offset_text += 18
            
            # Dimensions
            if real_w_mm is not None:
                dims_text = "{:.0f}x{:.0f}mm ({})".format(real_w_mm, real_h_mm, dim_method)
                cv2.putText(frame, dims_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                y_offset_text += 18
            
            # Log information
            if has_pos_info:
                simple_log_info = "Label: {}, X: {}, Y: {}".format(display_label, rel_x, rel_y)
                log_info_list.append(simple_log_info)
                
        except Exception as e:
            print("Error processing detection {}: {}".format(i, e))
    
    return robot_head, robot_tail, balls, walls, log_info_list

# SIMPLIFIED PROCESSOR FOR COMPATIBILITY
def process_detections(results, model):
    robot_head, robot_tail, balls, walls, _ = process_detections_and_draw(results, model, np.zeros((480, 640, 3), dtype=np.uint8))
    return robot_head, robot_tail, balls, 0 