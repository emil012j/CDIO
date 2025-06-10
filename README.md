## Start af systemet:
- `best.pt` YOLO model fil
- EV3 robot skal være på samme netværk som PC
- Opdater evt robot IP i `src/config/settings.py`
- Opdater evt camera source efter det usb man brger `src/config/settings.py` 

### ved opdatering af filer:
```bash
scp robot_app.py robot@192.168.8.97:/home/robot/
scp -r src/ robot@192.168.8.97:/home/robot/
```

### 1. På Robot:
start
```bash
python3 robot_app.py
```

### 2. På PC:
start
```bash
python vision_app.py
```
