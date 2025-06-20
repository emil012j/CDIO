## Starting the system:
- `best.pt` YOLO model file
- EV3 robot must be on the same network as PC
- Update robot IP if needed in `src/config/settings.py`
- Update camera source if needed based on the USB port used `src/config/settings.py` 

### When updating files:
```bash
scp robot_app.py robot@192.168.65.158:/home/robot/
scp -r src/ robot@192.168.65.158:/home/robot/
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
