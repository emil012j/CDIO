# -*- coding: utf-8 -*-
"""
YOLO model loading, objekt detektion og resultater
"""

import cv2
import numpy as np
from ultralytics import YOLO
import math
import os
from ..config.settings import *

#finder højde og bredde af objektet
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

#checker om det er en egg
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


#beregner skaleringsfaktor baseret på det røde kors
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
                        scale = CROSS_DIAMETER_MM / cross_obb_size_px
                        return float(scale)  # Beregner scale factor baseret på cross størrelse
                        
    except Exception: 
        return None

# Finder positionen af det røde kryds i billedet
def get_cross_position(results, model):
    try:
        if results is None or not hasattr(results, 'obb') or results.obb is None:
            return None
        cross_id_target = get_class_id('cross', model)
        if cross_id_target is None:
            return None
        for i in range(len(results.obb.cls)):
            cls_id = int(results.obb.cls[i])
            if cls_id == cross_id_target:
                if hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None and len(results.obb.xyxy) > i:
                    box = results.obb.xyxy[i].cpu().numpy()
                    x_center = (box[0] + box[2]) / 2
                    y_center = (box[1] + box[3]) / 2
                    return (x_center, y_center)
        return None
    except Exception:
        return None
    
#beregner vinklen på objektet
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

 #loader YOLO model
def load_yolo_model(model_path=MODEL_PATH):
    try:
        if not os.path.exists(model_path): 
            print("KAN IKKE FINDES {}".format(model_path))
            return None
        print("LOADER YOLO MODEL...")
        model = YOLO(model_path)  # Loader YOLO model (best.pt)
        if get_class_id('cross', model) is None: 
            print("KRYDSER IKKE FUNDET")
        return model
    except Exception as e: 
        print("ERROR loading model: {}".format(e))
        return None
    
#DETECTER OBJEKTER I KAMERAET 
def run_detection(model, frame):
    try:
        results_list = model(frame, imgsz=640, conf=CONFIDENCE_THRESHOLD, verbose=False)  # Kører detection på kamera frames
        if isinstance(results_list, list) and len(results_list) > 0:
            return results_list[0]
        return None
    except Exception: 
        return None
    
#PROCESSER OG TEGNER DETEKTERINGERNE SOM I DEN GAMLE FIL
def process_detections_and_draw(results, model, frame, scale_factor=None):
    robot_head = None
    robot_tail = None
    balls = []
    log_info_list = []
    
    # Check if we have valid detections
    if results is None or not hasattr(results, 'obb') or results.obb is None:
        return robot_head, robot_tail, balls, log_info_list
    if not hasattr(results.obb, 'cls') or results.obb.cls is None:
        return robot_head, robot_tail, balls, log_info_list
    
    try: 
        num_detections = len(results.obb.cls)
    except TypeError: 
        return robot_head, robot_tail, balls, log_info_list
    
    if num_detections == 0:
        return robot_head, robot_tail, balls, log_info_list
    
    # Tegn centerlinjer som i den gamle fil
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
            
            # Hent position information som i den gamle fil
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
            elif hasattr(results.obb, 'xywh') and results.obb.xywh is not None and len(results.obb.xywh) > i:
                cx_wh, cy_wh, _, _ = results.obb.xywh[i]
                cx_calc, cy_calc = int(cx_wh), int(cy_wh)
                x1 = cx_calc - 50
                y1 = cy_calc - 50
                x2 = cx_calc + 50
                y2 = cy_calc + 50
                has_pos_info = True
                
            if not has_pos_info:
                continue
                
            # Beregn real world dimensions som i den gamle fil
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
            
            # Egg detection som i den gamle fil
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
            
            # Beregn relative koordinater fra center
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
            
            # Gem balls (ikke eggs)
            if "ball" in display_label.lower() and "egg" not in display_label.lower():
                balls.append((cx_calc, cy_calc))
            
            # Tegn OBB eller rectangle som i den gamle fil
            color = CLASS_COLORS.get(display_label, (128, 128, 128))
            if obb_points is not None:
                cv2.polylines(frame, [obb_points], isClosed=True, color=color, thickness=2)
            elif x2 > x1:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            
            # Tegn labels som i den gamle fil
            y_offset_text = y1 + 20
            label_conf_text = "{} ({:.2f})".format(display_label, conf)
            cv2.putText(frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
            cv2.putText(frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            
            if is_egg:
                reason_text = "EGG ({})".format(egg_reason)
                cv2.putText(frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
                cv2.putText(frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
            
            # Koordinat tekst
            coord_text = "[X:{}, Y:{}]".format(rel_x, rel_y)
            cv2.putText(frame, coord_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            y_offset_text += 18
            
            # Dimensioner
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
    
    return robot_head, robot_tail, balls, log_info_list

#SIMPLIFIED PROCESSER FOR COMPATIBILITY  
def process_detections(results, model):
    robot_head, robot_tail, balls, _ = process_detections_and_draw(results, model, np.zeros((480, 640, 3), dtype=np.uint8))
    return robot_head, robot_tail, balls, 0 