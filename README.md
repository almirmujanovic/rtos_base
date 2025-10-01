# Semi-autonomous RC Car with ESP32, Arduino and Computer Vision

## Overview
This project implements a semi-autonomous RC vehicle with modular hardware and software architecture.  
The system combines:
- Arduino UNO R4 Minima as the sensor and actuator acquisition module  
- Freenove ESP32-S3 CAM (OV2640 camera) as the central controller with FreeRTOS, handling communication and video streaming  
- Python Dashboard App (PySide6) for FPV control, telemetry, and autonomous driving (via OpenCV + YOLOv8n)  

The car supports both manual mode (PlayStation DualShock 4 controller) and autonomous mode (lane following, traffic sign detection, obstacle avoidance).  

---

## Hardware
- Arduino UNO R4 Minima  
- Freenove ESP32-S3 CAM (OV2640 camera)  
- Sensors:  
  - 3× HC-SR04 ultrasonic (front-left, front-right, rear)  
  - 1× VL53L0X ToF (front-center)  
- Actuators:  
  - 2× TT DC motors (rear wheels, TB6612FNG driver)  
  - 1× SG90 servo (front steering)  
- Power supply:  
  - 18650 Li-ion cells (battery pack)  
  - 3× MP1584 buck regulators (6 V motors, 5 V logic/servo/ESP32/Arduino)  

---

## Software Architecture
- Arduino UNO R4  
  - Collects sensor data (HC-SR04, VL53L0X)  
  - Controls motors (PWM) and servo steering  
  - Communicates with ESP32 via UART  

- ESP32-S3 CAM (FreeRTOS)  
  - Manages system tasks: Wi-Fi, MQTT client, UART communication, RTSP server  
  - Streams live video (VGA, adaptive quality)  
  - Publishes/receives telemetry and commands over MQTT  

- Python Dashboard (PySide6)  
  - FPV interface: video stream, telemetry, connection status  
  - Manual driving with DualShock 4 controller  
  - Autonomous driving with OpenCV lane detection and YOLOv8n object detection  
  - Safety logic: stops vehicle if telemetry is delayed (>1 s)  

---

## Running the System
1. Start the Mosquitto MQTT broker on your laptop (default port 1883).  
2. Connect ESP32 to your Wi-Fi hotspot (tested with Intel Wi-Fi 6 AX201).  
3. Upload Arduino code to UNO R4 (sensor acquisition + motor/servo control).  
4. Flash ESP32 firmware (FreeRTOS + RTSP server + MQTT client).  
5. Run Python Dashboard App with the following command:  

```bash
python .\main.py --rtsp rtsp://192.168.x.x:8544/mjpeg/1 \
                 --rtp-port 5000 \
                 --flip-vert \
                 --mqtt-host 192.168.137.1 \
                 --mqtt-port 1883 \
                 --device cpu \
                 --imgsz 960 \
                 --conf 0.25
