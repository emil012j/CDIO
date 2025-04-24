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
import math
import os

MODEL_PATH = "best.pt"
SOURCE = 1
RESOLUTION = (1280, 720)
CONF_THRESH_DISPLAY = 0.35

BOLD_DIAMETER_MM = 40
CROSS_DIAMETER_MM = 200.0
EGG_SIZE_THRESHOLD_MM = 58.0

CLASS_COLORS = { "cross": (0, 255, 0), "egg": (255, 0, 0), "orange ball": (0, 140, 255), "white ball": (255, 255, 255), "robothead": (0, 0, 255), "robottail": (255, 0, 255) }
ORIENTED_OBJECTS = ["robothead", "robottail", "egg", "cross"]
EGG_OBB_ASPECT_RATIO_THRESHOLD = 1.3

def get_obb_dimensions_from_obb_results(obb_results, index):
    try:
        if hasattr(obb_results, 'wh') and obb_results.wh is not None and len(obb_results.wh) > index:
             wh = obb_results.wh[index].cpu().numpy(); w, h = min(wh), max(wh); return float(w), float(h)
        elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
            points = obb_results.xyxyxyxy[index].cpu().numpy().reshape(4, 2)
            side1=np.linalg.norm(points[0]-points[1]); side2=np.linalg.norm(points[1]-points[2])
            w, h = min(side1, side2), max(side1, side2); return float(w), float(h)
        else: return None, None
    except Exception: return None, None

def is_likely_egg_by_obb_shape_idx(obb_results, index):
    try:
        obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(obb_results, index)
        if obb_w_px is not None and obb_h_px is not None and obb_w_px > 1e-3:
            aspect_ratio = obb_h_px / obb_w_px
            return aspect_ratio >= EGG_OBB_ASPECT_RATIO_THRESHOLD
        else: return False
    except Exception: return False

def get_class_id(class_name_lower, model):
    for idx, name in model.names.items():
        if name.lower() == class_name_lower: return idx
    return None

def calculate_scale_factor(results, model):
    try:
        if results is None or not hasattr(results, 'obb') or results.obb is None: return None
        if not hasattr(results.obb, 'cls') or results.obb.cls is None: return None
        num_obb_boxes = len(results.obb.cls)
        if num_obb_boxes == 0: return None
        cross_id_target = get_class_id('cross', model)
        if cross_id_target is None: return None
        for i in range(num_obb_boxes):
             cls_id = int(results.obb.cls[i])
             conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
             if cls_id == cross_id_target and conf >= CONF_THRESH_DISPLAY:
                obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(results.obb, i)
                if obb_w_px is not None and obb_h_px is not None:
                    cross_obb_size_px = (obb_w_px + obb_h_px) / 2.0
                    if cross_obb_size_px > 1: return CROSS_DIAMETER_MM / cross_obb_size_px
        return None
    except Exception: return None

def calculate_orientation(obb_results, index, label):
    try:
        if hasattr(obb_results, 'angle') and obb_results.angle is not None and len(obb_results.angle)>index and obb_results.angle[index] is not None: return float(obb_results.angle[index])
        elif hasattr(obb_results, 'xyxyxyxy') and obb_results.xyxyxyxy is not None and len(obb_results.xyxyxyxy) > index:
           points=obb_results.xyxyxyxy[index].cpu().numpy().reshape(4,2); s1=np.linalg.norm(points[0]-points[1]); s2=np.linalg.norm(points[1]-points[2])
           if s1>s2: dx=points[1][0]-points[0][0]; dy=points[1][1]-points[0][1]
           else: dx=points[2][0]-points[1][0]; dy=points[2][1]-points[1][1]
           return math.degrees(math.atan2(-dy, dx))
        elif hasattr(obb_results, 'xyxy') and obb_results.xyxy is not None and len(obb_results.xyxy) > index:
            x1,y1,x2,y2=map(int,obb_results.xyxy[index]); w=x2-x1; h=y2-y1
            if h>0 and w>0 and label in ["egg","cross"] and (w/h>1.2 or h/w>1.2): return 90.0 if h>w else 0.0
        return 0.0
    except Exception: return 0.0

def draw_robot_tangent_line(frame, tail_pos, head_pos, color=(0,255,255), thickness=1, line_length=2000):
    try:
        if tail_pos is None or head_pos is None: return
        dx=head_pos[0]-tail_pos[0]; dy=head_pos[1]-tail_pos[1]
        if abs(dx)<1e-6 and abs(dy)<1e-6: return
        length=math.sqrt(dx*dx+dy*dy);
        if length<1e-6: return
        norm_dx=dx/length; norm_dy=dy/length
        sx=int(tail_pos[0]); sy=int(tail_pos[1])
        ex=int(head_pos[0]+norm_dx*line_length); ey=int(head_pos[1]+norm_dy*line_length)
        cv2.line(frame, (sx,sy), (ex,ey), color, thickness)
    except Exception: pass

def main():
    try:
        if not os.path.exists(MODEL_PATH): return
        model = YOLO(MODEL_PATH)
        if get_class_id('cross', model) is None: pass
    except Exception: return

    cap = cv2.VideoCapture(SOURCE, cv2.CAP_DSHOW)
    if not cap.isOpened(): cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened(): return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0]); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0); cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25); cap.set(cv2.CAP_PROP_EXPOSURE, -5)

    last_print_time = time.time(); print_interval = 5

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: time.sleep(0.1); continue

            results = None
            try:
                results_list = model(frame, imgsz=640, conf=CONF_THRESH_DISPLAY, verbose=False)
                if isinstance(results_list, list) and len(results_list) > 0:
                    results = results_list[0]
            except Exception: pass

            display_frame = frame.copy()

            process_detections = False
            num_detections = 0
            if results is not None and hasattr(results, 'obb') and results.obb is not None and hasattr(results.obb, 'cls') and results.obb.cls is not None:
                try: num_detections = len(results.obb.cls); process_detections = num_detections > 0
                except TypeError: pass

            cv2.line(display_frame, (RESOLUTION[0]//2,0), (RESOLUTION[0]//2,RESOLUTION[1]), (0,255,0), 1)
            cv2.line(display_frame, (0,RESOLUTION[1]//2), (RESOLUTION[0],RESOLUTION[1]//2), (0,255,0), 1)

            scale = None
            if process_detections: scale = calculate_scale_factor(results, model)

            robot_head = None; robot_tail = None
            log_info_list = []
            eggs_classified = 0; white_balls_remained = 0

            if process_detections:
                for i in range(num_detections):
                    try:
                        cls_id = int(results.obb.cls[i])
                        conf = float(results.obb.conf[i]) if hasattr(results.obb, 'conf') else 0.0
                        if cls_id < 0 or cls_id >= len(model.names): continue
                        original_label = model.names[cls_id]; display_label = original_label

                        x1, y1, x2, y2, cx_calc, cy_calc = 0, 0, 0, 0, 0, 0
                        has_pos_info = False; obb_points = None
                        if hasattr(results.obb, 'xyxyxyxy') and results.obb.xyxyxyxy is not None and len(results.obb.xyxyxyxy) > i:
                            obb_points = results.obb.xyxyxyxy[i].cpu().numpy().reshape(4, 2).astype(np.int32)
                            x_coords=obb_points[:,0]; y_coords=obb_points[:,1]
                            x1,y1=np.min(x_coords),np.min(y_coords); x2,y2=np.max(x_coords),np.max(y_coords)
                            cx_calc,cy_calc=np.mean(x_coords,axis=0).astype(int),np.mean(y_coords,axis=0).astype(int)
                            has_pos_info = True
                        elif hasattr(results.obb, 'xyxy') and results.obb.xyxy is not None and len(results.obb.xyxy) > i:
                             x1,y1,x2,y2=map(int,results.obb.xyxy[i]); cx_calc,cy_calc=(x1+x2)//2,(y1+y2)//2; has_pos_info=True
                        elif hasattr(results.obb, 'xywh') and results.obb.xywh is not None and len(results.obb.xywh) > i:
                             cx_wh,cy_wh,_,_=results.obb.xywh[i]; cx_calc,cy_calc=int(cx_wh),int(cy_wh)
                             x1=cx_calc-50; y1=cy_calc-50; x2=cx_calc+50; y2=cy_calc+50; has_pos_info=True
                        if not has_pos_info: continue

                        real_w_mm, real_h_mm, dim_method = None, None, "N/A"
                        if scale is not None:
                            obb_w_px, obb_h_px = get_obb_dimensions_from_obb_results(results.obb, i)
                            if obb_w_px is not None:
                                real_w_mm=obb_w_px*scale; real_h_mm=obb_h_px*scale; dim_method="OBB"
                            elif x2 > x1 and y2 > y1:
                                w_px=x2-x1; h_px=y2-y1; real_w_mm=w_px*scale; real_h_mm=h_px*scale; dim_method="xyxy"

                        is_egg = False; egg_reason = ""
                        if original_label == "white ball":
                            is_egg_shape = is_likely_egg_by_obb_shape_idx(results.obb, i)
                            if is_egg_shape: egg_reason = "Shape"
                            is_egg_size = False
                            if real_w_mm is not None and (real_w_mm > EGG_SIZE_THRESHOLD_MM or real_h_mm > EGG_SIZE_THRESHOLD_MM):
                                is_egg_size = True
                                if egg_reason: egg_reason += "+Size"
                                else: egg_reason = "Size"
                            is_egg = is_egg_shape or is_egg_size
                            if is_egg: display_label = "egg"; eggs_classified += 1
                            else: white_balls_remained += 1

                        rel_x = cx_calc - RESOLUTION[0] // 2; rel_y = RESOLUTION[1] // 2 - cy_calc
                        orientation_deg = 0.0
                        if display_label in ORIENTED_OBJECTS:
                            orientation_deg = calculate_orientation(results.obb, i, display_label)
                            if display_label == "robothead": robot_head = {"pos": (cx_calc, cy_calc), "orientation": orientation_deg}
                            elif display_label == "robottail": robot_tail = {"pos": (cx_calc, cy_calc), "orientation": orientation_deg}

                        color = CLASS_COLORS.get(display_label, (128, 128, 128))
                        y_offset_text = y1 + 20

                        if obb_points is not None: cv2.polylines(display_frame, [obb_points], isClosed=True, color=color, thickness=2)
                        elif x2 > x1: cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)

                        label_conf_text = f"{display_label} ({conf:.2f})"
                        cv2.putText(display_frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3);
                        cv2.putText(display_frame, label_conf_text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

                        if is_egg:
                             reason_text = f"EGG ({egg_reason})"
                             cv2.putText(display_frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
                             cv2.putText(display_frame, reason_text, (x1, y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

                        coord_text = f"[X:{rel_x}, Y:{rel_y}]"
                        cv2.putText(display_frame, coord_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1);
                        y_offset_text += 18

                        if real_w_mm is not None:
                             dims_text = f"{real_w_mm:.0f}x{real_h_mm:.0f}mm ({dim_method})"
                             cv2.putText(display_frame, dims_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1); y_offset_text += 18

                        # <<< VINKEL TEKST FJERNET HER >>>
                        # if display_label in ORIENTED_OBJECTS:
                        #     angle_text = f"Ang:{orientation_deg:.0f}"
                        #     cv2.putText(display_frame, angle_text, (x1, y_offset_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1); y_offset_text += 18

                        if has_pos_info:
                            simple_log_info = f"Label: {display_label}, X: {rel_x}, Y: {rel_y}"
                            log_info_list.append(simple_log_info)

                    except Exception: pass


            if robot_head and robot_tail:
                 try:
                    dx=robot_head["pos"][0]-robot_tail["pos"][0]; dy=robot_head["pos"][1]-robot_tail["pos"][1]
                    robot_angle_deg = math.degrees(math.atan2(-dy, dx))
                    cv2.line(display_frame, robot_tail["pos"], robot_head["pos"], (0, 255, 255), 2)
                    draw_robot_tangent_line(display_frame, robot_tail["pos"], robot_head["pos"], (0, 255, 255), 1, 2000)
                    robot_info_text = f"Robot Angle: {robot_angle_deg:.1f}"
                    cv2.putText(display_frame, robot_info_text, (10, RESOLUTION[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                 except Exception: pass


            if time.time() - last_print_time > print_interval:
                print(f"\n--- Objekter koordinater @ {time.strftime('%H:%M:%S')} ---")
                if log_info_list:
                    for info in log_info_list: print(info)
                else:
                    print(f" Ingen objekter detekteret over tærsklen ({CONF_THRESH_DISPLAY:.2f}).")

                last_print_time = time.time()


            cv2.putText(display_frame, f"Conf: {CONF_THRESH_DISPLAY:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


            cv2.imshow("YOLO OBB Detection", display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt: pass
    except Exception: pass
    finally:
        if cap.isOpened(): cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()