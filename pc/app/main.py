from __future__ import annotations
import sys
import argparse
from PySide6.QtWidgets import QApplication
from config import MqttConfig, VideoConfig, YoloConfig, ImageCaptureConfig
from ui.main_window import MainWindow

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--rtsp', type=str, default='', help='rtsp://HOST:PORT/mjpeg/1 (leave empty to run without video)')
    p.add_argument('--mqtt-host', type=str, default='127.0.0.1')
    p.add_argument('--mqtt-port', type=int, default=1883)
    p.add_argument('--rtp-port', type=int, default=5000, help='local RTP port for MJPEG (client_port)')
    p.add_argument('--flip-vert', action='store_true', help='flip video vertically (ESP32 quirk)')
    p.add_argument('--device', type=str, default='0')
    p.add_argument('--imgsz', type=int, default=960)
    p.add_argument('--conf', type=float, default=0.25)
    return p.parse_args()



if __name__ == '__main__':
    args = parse_args()
    app = QApplication(sys.argv)

    vcfg = VideoConfig(rtsp_url=args.rtsp, rtp_port=args.rtp_port, flip_vertical=args.flip_vert)
    mcfg = MqttConfig(host=args.mqtt_host, port=args.mqtt_port)
    ycfg = YoloConfig(device=args.device, imgsz=args.imgsz, conf=args.conf)
    icfg = ImageCaptureConfig()

    win = MainWindow(vcfg, mcfg, ycfg, icfg)
    win.show()
    sys.exit(app.exec())