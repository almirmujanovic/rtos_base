from __future__ import annotations
import time, math, numpy as np, cv2, os
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap, QPalette, QColor
from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QSlider, QSpinBox, QFrame, QStyleFactory,
    QComboBox, QProgressBar
)
from config import VideoConfig, MqttConfig, YoloConfig
from services.mqtt_service import MqttService
from services.joystick_service import JoystickService
from services.yolo_service import YoloWorker
from services.control_logic import decide_command
from services.rtp_mjpeg_client import RtspWorker
from ui.widgets import CarSensorView


def bgr_to_qimage(bgr):
    rgb=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); h,w,ch=rgb.shape
    return QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888).copy()

class MainWindow(QWidget):
    MODES = ["MANUAL", "FREE ROAM", "AUTONOMOUS"]

    def __init__(self, vcfg:VideoConfig, mcfg:MqttConfig, ycfg:YoloConfig):
        super().__init__()
        self.setWindowTitle('RC Car Control – PySide6')
        self.resize(1280,800)
        self._apply_dark()

        # State
        self.mode = "MANUAL"
        self.last_sensors={'vl53':0,'hcsr04_1':0,'hcsr04_2':0,'hcsr04_3':0}
        self.last_joy={'speed':0,'angle':90}
        self.last_diag={'fps':0.0,'seq_err':0}
        self.yolo_ready=False
        self.last_frame_size=(640,480)
        self.emergency_stop = False  # Track emergency state

        # Services - FIXED: Use consistent naming
        self.mqtt_service = MqttService(mcfg)
        self.mqtt_service.start()
        self.mqtt_service.sensorsUpdated.connect(self._on_sensors)
        self.mqtt_service.statusUpdated.connect(self._on_status)
        self.mqtt_service.connectedChanged.connect(self._on_mqtt)
        self.mqtt_service.emergencyStopChanged.connect(self._on_emergency_stop)

        self.joy=JoystickService(); self.joy.joystickChanged.connect(self._on_joy)

        self.rtsp=RtspWorker(vcfg.rtsp_url, rtp_port=vcfg.rtp_port, flip_vertical=vcfg.flip_vertical)
        self.rtsp.frameReady.connect(self._on_frame)
        self.rtsp.diag.connect(self._on_vdiag)
        self.rtsp.connectedChanged.connect(self._on_vid)
        self.rtsp.start()

        self.yolo=YoloWorker(ycfg)
        self.yolo.modelReady.connect(self._on_yolo_ready)
        self.yolo.annotatedReady.connect(self._on_annotated)

        # ==== UI LAYOUT ======================================================
        # Top bar (mode selector + YOLO config)
        self.mode_combo=QComboBox(); self.mode_combo.addItems(self.MODES); self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.sld_conf=QSlider(Qt.Horizontal); self.sld_conf.setRange(5,90); self.sld_conf.setValue(int(ycfg.conf*100)); self.sld_conf.valueChanged.connect(self._on_conf)
        self.sp_imgsz=QSpinBox(); self.sp_imgsz.setRange(320,1280); self.sp_imgsz.setSingleStep(64); self.sp_imgsz.setValue(ycfg.imgsz); self.sp_imgsz.valueChanged.connect(self._on_imgsz)
        topbar=QHBoxLayout();
        title = QLabel('RC CONTROL • DASHBOARD'); title.setObjectName("appTitle")
        topbar.addWidget(title); topbar.addStretch(1)
        topbar.addWidget(QLabel('MODE')); topbar.addWidget(self.mode_combo)
        topbar.addSpacing(16)
        topbar.addWidget(QLabel('YOLO CONF')); topbar.addWidget(self.sld_conf)
        topbar.addWidget(QLabel('IMGSZ')); topbar.addWidget(self.sp_imgsz)

        # Left column: Video (top-left quarter feel) + Car Sensor view below
        self.video_label=QLabel('VIDEO'); self.video_label.setFrameStyle(QFrame.StyledPanel)
        self.video_label.setMinimumSize(520,292)  # ~quarter on 1080p
        self.video_label.setAlignment(Qt.AlignCenter)

        self.car_view=CarSensorView()
        # Optional: if you place a better top-down car PNG, uncomment & set path:
        # p = "assets/car_top.png"
        # if os.path.exists(p): self.car_view.set_car_image(p)

        left_col=QVBoxLayout();
        left_col.addWidget(self.video_label,1)
        left_col.addWidget(self.car_view,1)

        # Right column: Diagnostics + Drive HUD
        self.grp_diag=QGroupBox('DIAGNOSTICS')
        diag_lay=QVBoxLayout(self.grp_diag)
        self.lbl_status=QLabel('Status: …')
        self.lbl_video=QLabel('Video: …')
        self.lbl_mqtt=QLabel('MQTT: …')
        self.lbl_yolo=QLabel('YOLO: …')
        self.lbl_diag=QLabel('Perf: …')
        self.lbl_arduino=QLabel('Arduino: …')  # NEW: Arduino status display
        for w in (self.lbl_status,self.lbl_video,self.lbl_mqtt,self.lbl_yolo,self.lbl_diag,self.lbl_arduino):
            w.setObjectName("diagText")
        diag_lay.addWidget(self.lbl_status)
        diag_lay.addWidget(self.lbl_video)
        diag_lay.addWidget(self.lbl_mqtt)
        diag_lay.addWidget(self.lbl_yolo)
        diag_lay.addWidget(self.lbl_arduino)  # NEW: Arduino status
        diag_lay.addWidget(self.lbl_diag)
        diag_lay.addStretch(1)

        self.grp_hud=QGroupBox('DRIVE HUD')
        hud_lay=QGridLayout(self.grp_hud)
        self.lbl_speed=QLabel('Speed: +000 (-255..+255)')
        self.pb_speed=QProgressBar(); self.pb_speed.setRange(0,255); self.pb_speed.setObjectName("barPrimary")
        self.lbl_angle=QLabel('Angle: 090° (40..140)')  # UPDATED: New range
        self.pb_angle=QProgressBar(); self.pb_angle.setRange(0,100); self.pb_angle.setObjectName("barSecondary")  # UPDATED: New range
        self.lbl_cmd=QLabel('CMD: …'); self.lbl_cmd.setObjectName("cmdText")
        hud_lay.addWidget(self.lbl_speed,0,0); hud_lay.addWidget(self.pb_speed,0,1)
        hud_lay.addWidget(self.lbl_angle,1,0); hud_lay.addWidget(self.pb_angle,1,1)
        hud_lay.addWidget(self.lbl_cmd,2,0,1,2)

        right_col=QVBoxLayout(); right_col.addWidget(self.grp_diag,2); right_col.addWidget(self.grp_hud,1); right_col.addStretch(1)

        # Root layout
        root=QVBoxLayout(self)
        root.addLayout(topbar)
        body=QHBoxLayout(); body.addLayout(left_col,1); body.addLayout(right_col,1)
        root.addLayout(body,1)

        # periodic HUD refresh (even if no new frames)
        self.hud_timer=QTimer(self); self.hud_timer.setInterval(200); self.hud_timer.timeout.connect(self._refresh_hud); self.hud_timer.start()

        # Initial "no data" state for the car widget
        self.car_view.set_data_available(False)

    # --- palette / style ---
    def _apply_dark(self):
        self.setStyle(QStyleFactory.create('Fusion'))
        pal=QPalette()
        pal.setColor(QPalette.Window, QColor(16,16,18))
        pal.setColor(QPalette.WindowText, QColor(230,230,235))
        pal.setColor(QPalette.Base, QColor(12,12,14))
        pal.setColor(QPalette.AlternateBase, QColor(24,24,26))
        pal.setColor(QPalette.Button, QColor(26,26,30))
        pal.setColor(QPalette.ButtonText, QColor(235,235,240))
        pal.setColor(QPalette.Text, QColor(240,240,245))
        pal.setColor(QPalette.Highlight, QColor(0,180,255))
        pal.setColor(QPalette.HighlightedText, QColor(0,0,0))
        self.setPalette(pal)

        # Pro-looking stylesheet (F1-diagnostics vibe) - UPDATED with emergency styles
        self.setStyleSheet("""
        #appTitle { font-size:18px; font-weight:700; letter-spacing: 2px; color:#E6E6EA; }
        QGroupBox {
            color:#AAB; font-weight:600; letter-spacing:1px; text-transform:uppercase;
            border: 1px solid #2A2E39; border-radius:8px; margin-top: 12px; padding: 10px;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
        QLabel { font-size:12px; color:#E6E6EA; }
        QLabel#diagText { font-family: "Consolas"; color:#B6FFEA; }
        QLabel#cmdText { font-family: "Consolas"; color:#9BD3FF; }
        QLabel#emergencyText { font-family: "Consolas"; color:#FF4444; font-weight: bold; }
        QProgressBar {
            background-color:#0E0F13; border:1px solid #2A2E39; border-radius:6px; text-align:center;
            color:#DDE; font-weight:600; min-height:18px;
        }
        QProgressBar#barPrimary::chunk { background: qlineargradient(y1:0, y2:1, stop:0 #0AE0FF, stop:1 #0066CC); border-radius:5px; }
        QProgressBar#barSecondary::chunk { background: qlineargradient(y1:0, y2:1, stop:0 #55FF99, stop:1 #1E8C4B); border-radius:5px; }
        QProgressBar#barEmergency::chunk { background: qlineargradient(y1:0, y2:1, stop:0 #FF4444, stop:1 #CC0000); border-radius:5px; }
        QComboBox, QSpinBox, QSlider::groove:horizontal {
            background:#14161A; border:1px solid #2A2E39; border-radius:6px; height: 10px;
        }
        QSlider::handle:horizontal { background:#00B4FF; width: 14px; border-radius:7px; margin:-5px 0; }
        """)

    # --- slots ---
    def _on_mqtt(self, ok:bool):
        # inform the car widget whether we have live sensor data
        self.car_view.set_data_available(ok)
        self._refresh_hud()

    def _on_vid(self, ok:bool):
        self._refresh_hud()

    def _on_vdiag(self, d:dict):
        self.last_diag.update(d); self._refresh_hud()

    def _on_sensors(self, d:dict):
        self.last_sensors.update(d)
        self.car_view.update_sensors(self.last_sensors)
        self._refresh_hud()

    def _on_status(self, status_msg: str):
        """Handle Arduino status messages - NEW"""
        self.lbl_arduino.setText(f"Arduino: {status_msg}")
        # Check for timeout or command messages
        if "timeout" in status_msg.lower():
            self.lbl_arduino.setObjectName("emergencyText")
        else:
            self.lbl_arduino.setObjectName("diagText")
        self.lbl_arduino.setStyle(self.lbl_arduino.style())  # Refresh style

    def _on_emergency_stop(self, is_emergency: bool):
        """Handle emergency stop state - NEW"""
        self.emergency_stop = is_emergency
        if is_emergency:
            # Change progress bar colors to red during emergency
            self.pb_speed.setObjectName("barEmergency")
            self.pb_angle.setObjectName("barEmergency")
            self.lbl_cmd.setObjectName("emergencyText")
            self.lbl_cmd.setText("🚨 EMERGENCY STOP - OBSTACLE DETECTED!")
        else:
            # Restore normal colors
            self.pb_speed.setObjectName("barPrimary")
            self.pb_angle.setObjectName("barSecondary")
            self.lbl_cmd.setObjectName("cmdText")
        
        # Refresh styles
        self.pb_speed.setStyle(self.pb_speed.style())
        self.pb_angle.setStyle(self.pb_angle.style())
        self.lbl_cmd.setStyle(self.lbl_cmd.style())

    def _on_yolo_ready(self, ok:bool):
        self.yolo_ready=ok; self._refresh_hud()

    def _on_joy(self, j:dict):
        self.last_joy=j
        self.pb_speed.setValue(abs(j['speed']))
        self.pb_angle.setValue(j['angle']-40)  # UPDATED: New range (40-140)
        self.lbl_speed.setText(f"Speed: {j['speed']:+4d} (-255..+255)")
        self.lbl_angle.setText(f"Angle: {j['angle']:03d}° (40..140)")  # UPDATED: New range
        
        # Only send commands if not in emergency stop
        if self.mode == "MANUAL" and not self.emergency_stop:
            self.mqtt_service.publish_move(j['speed'], j['angle'])  # FIXED: Use mqtt_service
        elif self.emergency_stop:
            # Override with stop command during emergency
            self.mqtt_service.publish_move(0, 90)  # Stop + center steering
            
        self._refresh_hud()

    def _on_mode_changed(self, txt:str):
        self.mode = txt
        self._refresh_hud()

    def _on_conf(self, v:int):
        self.yolo.cfg.conf=v/100.0
        self._refresh_hud()

    def _on_imgsz(self, v:int):
        self.yolo.cfg.imgsz=int(v)
        self._refresh_hud()

    def _refresh_hud(self):
        # FIXED: Use mqtt_service methods consistently
        mqtt_ok = self.mqtt_service.is_connected
        video_live = self.last_diag.get('fps',0)>0
        sensor_age = self.mqtt_service.get_sensor_age()
        sensor_live = self.mqtt_service.is_sensor_live()
        obstacles = self.mqtt_service.get_obstacles()
        
        self.lbl_status.setText(f"Mode: {self.mode}")
        self.lbl_video.setText(f"Video: {'LIVE' if video_live else 'OFFLINE'} | FPS {self.last_diag.get('fps',0):.1f} | seq_err {self.last_diag.get('seq_err',0)}")
        
        # Enhanced MQTT status with sensor info
        mqtt_status = "CONNECTED" if mqtt_ok else "OFFLINE"
        if mqtt_ok:
            sensor_status = f"| Sensors: {'LIVE' if sensor_live else 'STALE'} ({sensor_age:.1f}s ago)"
            if obstacles:
                mqtt_status += f" | 🚨 OBSTACLES: {', '.join(obstacles)}"
            else:
                mqtt_status += sensor_status
        self.lbl_mqtt.setText(f"MQTT: {mqtt_status}")
        
        self.lbl_yolo.setText(f"YOLO: {'READY' if self.yolo_ready else 'IDLE'} | conf {self.yolo.cfg.conf:.2f} | imgsz {self.yolo.cfg.imgsz}")

        # Enhanced command line text with emergency handling
        if self.emergency_stop:
            self.lbl_cmd.setText("🚨 EMERGENCY STOP - OBSTACLE DETECTED!")
        elif self.mode == "MANUAL":
            self.lbl_cmd.setText(f"CMD(MANUAL): speed={self.last_joy['speed']} angle={self.last_joy['angle']}")
        elif self.mode == "FREE ROAM":
            self.lbl_cmd.setText("CMD(FREE ROAM): sensor-based navigation (no YOLO)")
        else:
            self.lbl_cmd.setText('CMD(AUTONOMOUS): YOLO + sensors (decision shown only)')

    def _draw_overlay(self, frame):
        # Enhanced overlay with sensor age and emergency status
        h,w=frame.shape[:2]
        pa=frame.copy()
        cv2.rectangle(pa,(10,10),(int(w*0.72),90),(0,0,0),-1)
        blend=cv2.addWeighted(pa,0.30,frame,0.70,0)
        
        # Line 1: Mode, FPS, Sensor age
        sensor_age = self.mqtt_service.get_sensor_age()
        t1=f"MODE:{self.mode}   FPS:{self.last_diag.get('fps',0):.1f}   SENSOR_AGE:{sensor_age:.1f}s"
        
        # Line 2: Sensor values with emergency highlighting
        s=self.last_sensors
        t2=f"F:{s.get('vl53',0):3d}  FL:{s.get('hcsr04_1',0):3d}  FR:{s.get('hcsr04_2',0):3d}  B:{s.get('hcsr04_3',0):3d}"
        
        color1 = (255,255,255)
        color2 = (0,255,200) if not self.emergency_stop else (0,0,255)  # Red if emergency
        
        cv2.putText(blend,t1,(24,42),cv2.FONT_HERSHEY_SIMPLEX,0.8,color1,2)
        cv2.putText(blend,t2,(24,76),cv2.FONT_HERSHEY_SIMPLEX,0.8,color2,2)
        
        # Emergency warning overlay
        if self.emergency_stop:
            cv2.putText(blend,"🚨 EMERGENCY STOP",(int(w*0.75),50),cv2.FONT_HERSHEY_SIMPLEX,1.2,(0,0,255),3)
        
        return blend

    def _on_frame(self, frame_bgr):
        h,w=frame_bgr.shape[:2]
        self.last_frame_size=(w,h)
        base = self._draw_overlay(frame_bgr)

        if self.mode == "AUTONOMOUS":
            self.yolo.infer(base)
            img=bgr_to_qimage(base)
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            img=bgr_to_qimage(base)
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        # Enhanced FREE ROAM with emergency handling
        if self.mode == "FREE ROAM" and not self.emergency_stop:
            decision = decide_command([], w, h, self.last_sensors)
            self.lbl_cmd.setText(f"FREE ROAM decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']})")
            # Optionally send the decision to robot
            # speed = int(decision['throttle'] * 100)  # Scale as needed
            # angle = int(90 + decision['steer'] * 25)  # Scale and center
            # self.mqtt_service.publish_move(speed, angle)

    def _on_annotated(self, annotated_bgr, dets):
        img=bgr_to_qimage(annotated_bgr)
        self.video_label.setPixmap(QPixmap.fromImage(img).scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        w,h=self.last_frame_size
        try:
            if not self.emergency_stop:  # Only make decisions if not in emergency
                decision = decide_command(dets, w, h, self.last_sensors)
                if 'target' in decision:
                    self.lbl_cmd.setText(f"AUTO decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']}) target={decision['target']} conf={decision.get('conf',0):.2f}")
                else:
                    self.lbl_cmd.setText(f"AUTO decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']})")
                
                # Optionally send autonomous commands
                # speed = int(decision['throttle'] * 100)  # Scale as needed  
                # angle = int(90 + decision['steer'] * 25)  # Scale and center
                # self.mqtt_service.publish_move(speed, angle)
        except Exception:
            pass