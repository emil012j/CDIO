## Starting the system:
- `best.pt` YOLO model file
- EV3 robot must be on the same network as PC
- Update robot IP if needed in `src/config/settings.py`
- Update camera source if needed based on the USB port used `src/config/settings.py` 

### When updating files:
```bash
scp robot_app.py robot@192.168.158.158:/home/robot/
scp -r src/ robot@192.168.158.158:/home/robot/
```

### 1. On Robot:
start
```bash
python3 robot_app.py
```

### 2. On PC:
start
```bash
python vision_app.py
```

  # --- Cross avoidance flag reset (always runs) ---
            if just_avoided_cross:
                if cross_pos and robot_head:
                    head_x, head_y = robot_head["pos"]
                    cross_x, cross_y = cross_pos
                    dist_to_cross = ((head_x - cross_x) ** 2 + (head_y - cross_y) ** 2) ** 0.5
                    if dist_to_cross > 180:  # Use a larger threshold for reset
                        just_avoided_cross = False
                        cross_avoid_reset_time = None
                else:
                    if cross_avoid_reset_time is None:
                        cross_avoid_reset_time = time.time()
                    elif time.time() - cross_avoid_reset_time > 2:  # 2 seconds without seeing the cross
                        just_avoided_cross = False
                        cross_avoid_reset_time = None
            else:
                cross_avoid_reset_time = None

            # --- Cross avoidance logic (dynamic turn direction, using head position) ---
            if cross_pos and robot_head:
                head_x, head_y = robot_head["pos"]
                cross_x, cross_y = cross_pos
                dist_to_cross = ((head_x - cross_x) ** 2 + (head_y - cross_y) ** 2) ** 0.5
                if dist_to_cross <= 120 and not just_avoided_cross:
                    # Dynamic turn direction: turn away from cross
                    turn_direction = "right" if cross_x > head_x else "left"
                    if commander.can_send_command():
                        print(f"*** CLOSE TO CROSS - GOING BACKWARDS AND TURNING {turn_direction.upper()} ***")
                        commander.send_backward_command(distance=20)
                        time.sleep(1)
                        commander.send_turn_rotation_command(turn_direction, 0.5)
                        time.sleep(1)
                        commander.send_forward_command(distance=15)
                        time.sleep(1)
                        just_avoided_cross = True
                        continue