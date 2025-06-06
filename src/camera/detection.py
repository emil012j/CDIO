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
                        return CROSS_DIAMETER_MM / cross_obb_size_px  # Beregner scale factor baseret på cross størrelse
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
    
#PROCESSER DETEKTERINGERNE
def process_detections(results, model):
    robot_head = None
    robot_tail = None
    balls = []
    
    # Check if we have valid detections
    if results is None or not hasattr(results, 'obb') or results.obb is None:
        return robot_head, robot_tail, balls, 0
    if not hasattr(results.obb, 'cls') or results.obb.cls is None:
        return robot_head, robot_tail, balls, 0
    
    try: 
        num_detections = len(results.obb.cls)
    except TypeError: 
        return robot_head, robot_tail, balls, 0
    
    if num_detections == 0:
        return robot_head, robot_tail, balls, 0
    
    for i in range(num_detections):
        try:
            cls_id = int(results.obb.cls[i])
            conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
            if cls_id < 0 or cls_id >= len(model.names): 
                continue
            
            label = model.names[cls_id]
            
            #cx er midtpunktet på x-aksen og cy er midtpunktet på y-aksen
            cx_calc, cy_calc = 0, 0
            
            #xyxyxyxy er en liste med 4 punkter, hvor hvert punkt har x og y koordinater
            if hasattr(results.obb, 'xyxyxyxy') and results.obb.xyxyxyxy is not None:
                if len(results.obb.xyxyxyxy) > i:
                    obb_points = results.obb.xyxyxyxy[i].cpu().numpy().reshape(4, 2).astype(np.int32)
                    x_coords = obb_points[:,0]
                    y_coords = obb_points[:,1]
                    cx_calc = int(np.mean(x_coords))
                    cy_calc = int(np.mean(y_coords))
            elif hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None:
                if len(results.obb.xyxy) > i:
                    x1, y1, x2, y2 = map(int, results.obb.xyxy[i])
                    cx_calc, cy_calc = (x1+x2)//2, (y1+y2)//2
            else:
                continue
                
            # Finder robothead, robottail, bolde
            if label == "robothead":
                robot_head = {"pos": (cx_calc, cy_calc)}
            elif label == "robottail":
                robot_tail = {"pos": (cx_calc, cy_calc)}
            elif "ball" in label.lower() and "egg" not in label.lower():
                balls.append((cx_calc, cy_calc))
                
        except Exception as e:
            print("Error processing detection {}: {}".format(i, e))
    
    return robot_head, robot_tail, balls, num_detections 