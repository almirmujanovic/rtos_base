from __future__ import annotations
from dataclasses import dataclass

@dataclass
class MqttConfig:
    host: str = '127.0.0.1'
    port: int = 1883
    topic_sensors: str = '/robot/sensors'   # CSV "vl53,hcsr04_1,hcsr04_2,hcsr04_3"
    topic_status: str = '/robot/status'
    topic_commands: str = '/robot/commands' # "MOVE,<speed>,<angle>"

@dataclass
class VideoConfig:
    rtsp_url: str = ''              # if empty → app still runs with NO VIDEO placeholder
    rtp_port: int = 5000            # client_port for RTP (RTCP = rtp_port+1)
    flip_vertical: bool = True

@dataclass
class YoloConfig:
    model_path: str = 'yolov8n.pt'
    device: str = '0'   # '0' for CUDA GPU, or 'cpu'
    imgsz: int = 960
    conf: float = 0.25
    iou: float = 0.6