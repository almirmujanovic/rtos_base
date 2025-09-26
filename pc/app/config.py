from __future__ import annotations
from dataclasses import dataclass, field

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
    model_path: str = 'models/model2.pt'
    device: str = 'cpu'   # '0' for CUDA GPU, or 'cpu'
    imgsz: int = 512
    conf: float = 0.55
    iou: float = 0.5


# Add this to your existing config.py:

@dataclass
class ImageCaptureConfig:
    """Configuration for image capture service"""
    enabled: bool = True                  
    save_directory: str = "training_images"  # Directory to save images
    interval_ms: int = 1000                   # Save every 1000ms (1 FPS)
    auto_start: bool = False                 # Start capturing on app launch
    max_images: int = 0                      # 0 = unlimited
    jpeg_quality: int = 95                   # High quality for training data
  
@dataclass
class LaneConfig:
    """Lane detection configuration"""
    roi_top_frac: float = 0.6
    min_line_length: int = 10
    max_line_gap: int = 20
    theta_threshold: float = 3.0
    hsv_lower: list = field(default_factory=lambda: [0, 0, 120])
    hsv_upper: list = field(default_factory=lambda: [180, 50, 255])
    canny_low: int = 30
    canny_high: int = 100

@dataclass
class PIDConfig:
    """PID controller configuration"""
    kp: float = 1.0
    ki: float = 0.01
    kd: float = 0.4

@dataclass
class AutonomousConfig:
    """Autonomous mode configuration"""
    cmd_hz: int = 20
    speed: int = 45

# Servo konstante
CENTER_ANGLE: int = 90
ANGLE_MIN: int = 25
ANGLE_MAX: int = 155
ANGLE_RANGE: int = min(CENTER_ANGLE-ANGLE_MIN, ANGLE_MAX-CENTER_ANGLE)

LANE_ROI_TOP_FRAC = 0.4           # Bottom 60% for better curve detection
LANE_MIN_LINE_LENGTH = 8          # Shorter for curve segments
LANE_MAX_LINE_GAP = 15            # Smaller gaps for precision
LANE_THETA_THRESHOLD = 1.5        # Very sensitive for sharp curves
LANE_HSV_LOWER = [0, 0, 240]      # Very bright white
LANE_HSV_UPPER = [180, 20, 255]   # Low saturation for pure white
LANE_CANNY_LOW = 20               # Low for white paper edges
LANE_CANNY_HIGH = 60              # Moderate for clean detection

# PID 
PID_KP = 1.4                    
PID_KI = 0.015                  
PID_KD = 0.6        
AUTO_CMD_HZ = 30
AUTO_SPEED = 55