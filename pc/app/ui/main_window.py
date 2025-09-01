from __future__ import annotations
import time, math, numpy as np, cv2, os
import threading
import queue
from dataclasses import dataclass

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap, QPalette, QColor
from PySide6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QSlider, QSpinBox, QFrame, QStyleFactory,
    QComboBox, QProgressBar, QPushButton, QCheckBox
)

from config import VideoConfig, MqttConfig, YoloConfig, ImageCaptureConfig
from config import (
    CENTER_ANGLE, ANGLE_MIN, ANGLE_MAX, ANGLE_RANGE,
    AUTO_SPEED, LANE_ROI_TOP_FRAC, PID_KP, PID_KI, PID_KD, AUTO_CMD_HZ,
    LANE_MIN_LINE_LENGTH, LANE_MAX_LINE_GAP, LANE_THETA_THRESHOLD,
    LANE_HSV_LOWER, LANE_HSV_UPPER, LANE_CANNY_LOW, LANE_CANNY_HIGH
)

from services.mqtt_service import MqttService
from services.joystick_service import JoystickService
from services.yolo_service import YoloWorker
from services.control_logic import decide_command
from services.rtp_mjpeg_client import RtspWorker
from services.image_capture_service import ImageCaptureService
from services.pid_service import PID
from services.lane_service import LaneFollower
from ui.widgets import CarSensorView

@dataclass
class LaneResult:
    error: float
    debug_info: dict
    timestamp: float

class MainWindow(QWidget):
    MODES = ["MANUAL", "FREE ROAM", "AUTONOMOUS"]

    def __init__(self, vcfg: VideoConfig, mcfg: MqttConfig, ycfg: YoloConfig, icfg: ImageCaptureConfig):
        super().__init__()
        self.setWindowTitle('RC Car Control – PySide6')
        self.resize(1280, 800)
        self._apply_dark()

        # State
        self.mode = "MANUAL"
        self.last_sensors = {'vl53': 0, 'hcsr04_1': 0, 'hcsr04_2': 0, 'hcsr04_3': 0}
        self.last_joy = {'speed': 0, 'angle': 90}
        self.last_diag = {'fps': 0.0, 'seq_err': 0}
        self.yolo_ready = False
        self.last_frame_size = (640, 480)
        self.emergency_stop = False
        self.current_frame = None

        # ✅ FIXED: Lane processing setup BEFORE services
        self.lane_queue = queue.Queue(maxsize=2)
        self.lane_result_queue = queue.Queue(maxsize=5)
        self.lane_processing = False
        self.lane_thread = None
        self.latest_lane_result = LaneResult(error=0.0, debug_info={}, timestamp=0.0)

        # ✅ FIXED: Initialize lane service and PID
        # Update the lane service initialization section:

        # ✅ FIXED: Initialize lane service and PID
        # Update the lane service initialization:

        # ✅ FIXED: Initialize lane service for WHITE LANE FOLLOWING
        self.pid = PID(PID_KP, PID_KI, PID_KD, out_min=-1.0, out_max=1.0)
        self.lane = LaneFollower(
            roi_top_frac=0.65,           # ✅ Focus on bottom 35% where lanes are
            theta_threshold=2.5,         # ✅ More sensitive steering
            min_line_length=15,          # ✅ Longer lines for stability
            max_line_gap=25              # ✅ Allow gaps in lane markings
        )
        
        # ✅ Configure for WHITE LANE detection
        self.lane.set_white_hsv_range([0, 0, 200], [180, 30, 255])  # Bright white
        self.lane.set_canny_thresholds(40, 120)                     # Good edge detection
        
        # ✅ TUNE: Optimize for white lane accuracy
        self.lane.tune_for_speed()  # Best white detection
        
        # ✅ ALTERNATIVE: Tune for speed if needed
        # self.lane.tune_for_speed()   # Faster but less precise
        
        # ✅ Alternative tuning options:
        # self.lane.tune_for_accuracy()  # More precise but slower
        # self.lane.tune_for_curves()    # Better curved lane detection
        
        # Configure lane service
        
        self.auto_last_cmd_t = 0.0
        self.auto_dt = 1.0 / float(AUTO_CMD_HZ)
        self.show_lane_mask = False  # Disable OpenCV windows for performance

        # ✅ FIXED: Lane processing timer
        self.lane_timer = QTimer()
        self.lane_timer.timeout.connect(self._process_lane_commands)

        # Services - FIXED: Use consistent naming
        self.mqtt_service = MqttService(mcfg)
        self.mqtt_service.start()
        self.mqtt_service.sensorsUpdated.connect(self._on_sensors)
        self.mqtt_service.statusUpdated.connect(self._on_status)
        self.mqtt_service.connectedChanged.connect(self._on_mqtt)
        self.mqtt_service.emergencyStopChanged.connect(self._on_emergency_stop)

        # Image capture service
        self.image_capture = ImageCaptureService(
            save_directory=icfg.save_directory,
            interval_ms=icfg.interval_ms
        )
        self.image_capture.enabled = icfg.enabled
        self.image_capture.imageSaved.connect(self._on_image_saved)
        self.image_capture.captureStatusChanged.connect(self._on_capture_status_changed)
        self.image_capture.errorOccurred.connect(self._on_capture_error)

        # Joystick and video services
        self.joy = JoystickService()
        self.joy.joystickChanged.connect(self._on_joy)

        self.rtsp = RtspWorker(vcfg.rtsp_url, rtp_port=vcfg.rtp_port, flip_vertical=vcfg.flip_vertical)
        self.rtsp.frameReady.connect(self._on_frame)
        self.rtsp.diag.connect(self._on_vdiag)
        self.rtsp.connectedChanged.connect(self._on_vid)
        self.rtsp.start()

        self.yolo = YoloWorker(ycfg)
        self.yolo.modelReady.connect(self._on_yolo_ready)
        self.yolo.annotatedReady.connect(self._on_annotated)

        # ==== UI LAYOUT ======================================================
        # Top bar
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(self.MODES)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        
        self.sld_conf = QSlider(Qt.Horizontal)
        self.sld_conf.setRange(5, 90)
        self.sld_conf.setValue(int(ycfg.conf * 100))
        self.sld_conf.valueChanged.connect(self._on_conf)
        
        self.sp_imgsz = QSpinBox()
        self.sp_imgsz.setRange(320, 1280)
        self.sp_imgsz.setSingleStep(64)
        self.sp_imgsz.setValue(ycfg.imgsz)
        self.sp_imgsz.valueChanged.connect(self._on_imgsz)

        topbar = QHBoxLayout()
        title = QLabel('RC CONTROL • DASHBOARD')
        title.setObjectName("appTitle")
        topbar.addWidget(title)
        topbar.addStretch(1)
        topbar.addWidget(QLabel('MODE'))
        topbar.addWidget(self.mode_combo)
        topbar.addSpacing(16)
        topbar.addWidget(QLabel('YOLO CONF'))
        topbar.addWidget(self.sld_conf)
        topbar.addWidget(QLabel('IMGSZ'))
        topbar.addWidget(self.sp_imgsz)

        # Image capture controls
        if icfg.enabled:
            topbar.addSpacing(20)
            
            self.capture_enabled_cb = QCheckBox("📸 Capture")
            self.capture_enabled_cb.setChecked(self.image_capture.enabled)
            self.capture_enabled_cb.toggled.connect(self._on_capture_enabled)
            topbar.addWidget(self.capture_enabled_cb)
            
            self.capture_button = QPushButton("Start")
            self.capture_button.setMaximumWidth(60)
            self.capture_button.clicked.connect(self._toggle_capture)
            self.capture_button.setEnabled(self.image_capture.enabled)
            topbar.addWidget(self.capture_button)
            
            self.save_button = QPushButton("Save")
            self.save_button.setMaximumWidth(50)
            self.save_button.clicked.connect(self._manual_save)
            self.save_button.setEnabled(self.image_capture.enabled)
            topbar.addWidget(self.save_button)
            
            self.capture_status_label = QLabel("Ready")
            self.capture_status_label.setObjectName("captureStatus")
            self.capture_status_label.setMinimumWidth(80)
            topbar.addWidget(self.capture_status_label)

        # Left column: Video + Car Sensor view
        self.video_label = QLabel('VIDEO')
        self.video_label.setFrameStyle(QFrame.StyledPanel)
        self.video_label.setMinimumSize(520, 292)
        self.video_label.setAlignment(Qt.AlignCenter)

        self.car_view = CarSensorView()

        left_col = QVBoxLayout()
        left_col.addWidget(self.video_label, 1)
        left_col.addWidget(self.car_view, 1)

        # Right column: Diagnostics + Drive HUD
        self.grp_diag = QGroupBox('DIAGNOSTICS')
        diag_lay = QVBoxLayout(self.grp_diag)
        
        self.lbl_status = QLabel('Status: …')
        self.lbl_video = QLabel('Video: …')
        self.lbl_mqtt = QLabel('MQTT: …')
        self.lbl_yolo = QLabel('YOLO: …')
        self.lbl_diag = QLabel('Perf: …')
        self.lbl_arduino = QLabel('Arduino: …')

        for w in (self.lbl_status, self.lbl_video, self.lbl_mqtt, self.lbl_yolo, self.lbl_diag, self.lbl_arduino):
            w.setObjectName("diagText")

        diag_lay.addWidget(self.lbl_status)
        diag_lay.addWidget(self.lbl_video)
        diag_lay.addWidget(self.lbl_mqtt)
        diag_lay.addWidget(self.lbl_yolo)
        diag_lay.addWidget(self.lbl_arduino)

        if icfg.enabled:
            self.lbl_capture = QLabel('Capture: …')
            self.lbl_capture.setObjectName("diagText")
            diag_lay.addWidget(self.lbl_capture)

        diag_lay.addWidget(self.lbl_diag)
        diag_lay.addStretch(1)

        # Drive HUD
        self.grp_hud = QGroupBox('DRIVE HUD')
        hud_lay = QGridLayout(self.grp_hud)
        
        self.lbl_speed = QLabel('Speed: +000 (-255..+255)')
        self.pb_speed = QProgressBar()
        self.pb_speed.setRange(0, 255)
        self.pb_speed.setObjectName("barPrimary")
        
        self.lbl_angle = QLabel('Angle: 090° (40..140)')
        self.pb_angle = QProgressBar()
        self.pb_angle.setRange(0, 100)
        self.pb_angle.setObjectName("barSecondary")
        
        self.lbl_cmd = QLabel('CMD: …')
        self.lbl_cmd.setObjectName("cmdText")
        
        hud_lay.addWidget(self.lbl_speed, 0, 0)
        hud_lay.addWidget(self.pb_speed, 0, 1)
        hud_lay.addWidget(self.lbl_angle, 1, 0)
        hud_lay.addWidget(self.pb_angle, 1, 1)
        hud_lay.addWidget(self.lbl_cmd, 2, 0, 1, 2)

        right_col = QVBoxLayout()
        right_col.addWidget(self.grp_diag, 2)
        right_col.addWidget(self.grp_hud, 1)
        right_col.addStretch(1)

        # Root layout
        root = QVBoxLayout(self)
        root.addLayout(topbar)
        body = QHBoxLayout()
        body.addLayout(left_col, 1)
        body.addLayout(right_col, 1)
        root.addLayout(body, 1)

        # Periodic HUD refresh
        self.hud_timer = QTimer(self)
        self.hud_timer.setInterval(200)
        self.hud_timer.timeout.connect(self._refresh_hud)
        self.hud_timer.start()

        # Initial state
        self.car_view.set_data_available(False)

        # Auto-start capture if configured
        if icfg.enabled and icfg.auto_start:
            QTimer.singleShot(3000, self._start_capture_delayed)

    def bgr_to_qimage(self, bgr_frame):
        """Convert BGR frame to QImage for Qt display"""
        try:
            rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            return QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        except Exception as e:
            print(f"❌ bgr_to_qimage error: {e}")
            black_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            rgb_frame = cv2.cvtColor(black_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            return QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

    # ===== EVENT HANDLERS =====
    # Replace the _on_frame method:

    def _on_frame(self, frame_bgr):
        """Handle incoming video frame - FAST video display only"""
        h, w = frame_bgr.shape[:2]
        self.last_frame_size = (w, h)
        self.current_frame = frame_bgr.copy()

        # Feed frame to image capture
        if hasattr(self, 'image_capture'):
            self.image_capture.update_original_frame(frame_bgr)

        # Always show video immediately (no processing delay)
        overlay_frame = self._draw_overlay(frame_bgr)
        
        if hasattr(self, 'image_capture'):
            self.image_capture.update_overlay_frame(overlay_frame)

        # Display video immediately
        img = self.bgr_to_qimage(overlay_frame)
        self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        # ✅ PURE LANE FOLLOWING: Only process lanes in autonomous mode
        if self.mode == "AUTONOMOUS":
            self._queue_frame_for_lane_processing(frame_bgr)
            # ✅ REMOVED: No YOLO processing for pure lane following
            
        elif self.mode == "FREE ROAM" and not self.emergency_stop:
            # FREE ROAM logic (sensor-based only)
            decision = decide_command([], w, h, self.last_sensors)
            self.lbl_cmd.setText(f"FREE ROAM decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']})")

    def _queue_frame_for_lane_processing(self, frame_bgr):
        """Queue frame for asynchronous lane processing"""
        try:
            if not self.lane_queue.full():
                # Downsample for speed
                h, w = frame_bgr.shape[:2]
                small_frame = cv2.resize(frame_bgr, (w//2, h//2))
                self.lane_queue.put_nowait(small_frame)
                
                if not self.lane_processing:
                    self._start_lane_processing_thread()
        except queue.Full:
            pass

    def _start_lane_processing_thread(self):
        """Start lane processing thread"""
        if self.lane_thread is None or not self.lane_thread.is_alive():
            self.lane_processing = True
            self.lane_thread = threading.Thread(target=self._lane_processing_worker, daemon=True)
            self.lane_thread.start()

    # Add this method to the MainWindow class:

    def _show_lane_debug(self, debug_frame):
        """Show lane detection debug window"""
        if debug_frame is not None and self.show_lane_mask:
            cv2.imshow("White Lane Detection - 4 Panel Debug", debug_frame)
            cv2.waitKey(1)

    # Update the lane processing worker to show debug:
    def _lane_processing_worker(self):
        """Worker thread for lane processing"""
        while self.lane_processing and self.mode == "AUTONOMOUS":
            try:
                frame = self.lane_queue.get(timeout=0.1)
                
                start_time = time.time()
                lane_error, debug_frame = self.lane.compute_error(frame)
                processing_time = time.time() - start_time
                
                # ✅ Show debug window (if enabled)
                if debug_frame is not None:
                    cv2.imshow("White Lane Detection", debug_frame)
                    cv2.waitKey(1)
                
                debug_info = self.lane.get_debug_info()
                debug_info['processing_time_ms'] = processing_time * 1000
                
                result = LaneResult(
                    error=lane_error,
                    debug_info=debug_info,
                    timestamp=time.time()
                )
                
                if not self.lane_result_queue.full():
                    self.lane_result_queue.put_nowait(result)
                
                self.lane_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Lane processing worker error: {e}")
                
        self.lane_processing = False

    def _process_lane_commands(self):
        """Process lane commands at fixed rate (separate from video)"""
        if self.mode != "AUTONOMOUS":
            return

        try:
            # Get latest lane result
            while not self.lane_result_queue.empty():
                self.latest_lane_result = self.lane_result_queue.get_nowait()

            current_time = time.time()
            result_age = current_time - self.latest_lane_result.timestamp

            if result_age > 0.2:  # 200ms timeout
                lane_error = 0.0
                debug_info = {'detected_lines': 0, 'last_theta': 0.0, 'processing_time_ms': 0}
            else:
                lane_error = self.latest_lane_result.error
                debug_info = self.latest_lane_result.debug_info

            # Rate limiting
            if current_time - self.auto_last_cmd_t < self.auto_dt:
                return

            # PID control
            dt = current_time - self.auto_last_cmd_t if self.auto_last_cmd_t > 0 else self.auto_dt
            steering_output = self.pid.compute(lane_error, dt)
            
            # Throttle with safety
            throttle_speed = AUTO_SPEED
            front = self.last_sensors.get('vl53')
            if front is not None and 0 < front < 20:
                throttle_speed = 0

            # Convert to servo angle
            angle_offset = steering_output * ANGLE_RANGE
            target_angle = CENTER_ANGLE + int(angle_offset)
            target_angle = max(ANGLE_MIN, min(ANGLE_MAX, target_angle))

            # Send command
            if not self.emergency_stop:
                self.mqtt_service.publish_move(throttle_speed, target_angle)
                self.auto_last_cmd_t = current_time

            # Update HUD
            proc_time = debug_info.get('processing_time_ms', 0)
            self.lbl_cmd.setText(
                f"ASYNC LANE: error={lane_error:.3f}, pid={steering_output:.3f}, "
                f"lines={debug_info.get('detected_lines', 0)}, θ={debug_info.get('last_theta', 0):.2f}, "
                f"proc={proc_time:.1f}ms, age={result_age*1000:.0f}ms, speed={throttle_speed}, angle={target_angle}°"
            )

        except Exception as e:
            print(f"❌ Lane command processing error: {e}")
            if not self.emergency_stop:
                self.mqtt_service.publish_move(0, CENTER_ANGLE)

    def _on_annotated(self, annotated_bgr, dets):
        """Handle YOLO annotated frame"""
        img = self.bgr_to_qimage(annotated_bgr)
        self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        w, h = self.last_frame_size
        try:
            if not self.emergency_stop:
                decision = decide_command(dets, w, h, self.last_sensors)
                if 'target' in decision:
                    self.lbl_cmd.setText(f"AUTO decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']}) target={decision['target']} conf={decision.get('conf', 0):.2f}")
                else:
                    self.lbl_cmd.setText(f"AUTO decision → steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']})")
        except Exception:
            pass

    def _on_mode_changed(self, new_mode):
        """Handle mode change"""
        old_mode = self.mode
        self.mode = new_mode

        if new_mode == "AUTONOMOUS":
            print("🚗 Starting autonomous mode with async lane processing")
            lane_interval = int(1000 / AUTO_CMD_HZ)
            self.lane_timer.start(lane_interval)
            self.pid.reset()
        else:
            print(f"🛑 Stopping autonomous mode (was {old_mode})")
            self.lane_timer.stop()
            self.lane_processing = False
            
            # Clear queues
            while not self.lane_queue.empty():
                try:
                    self.lane_queue.get_nowait()
                except queue.Empty:
                    break
            while not self.lane_result_queue.empty():
                try:
                    self.lane_result_queue.get_nowait()
                except queue.Empty:
                    break

        self._refresh_hud()

    # ===== OTHER EVENT HANDLERS =====
    def _on_mqtt(self, ok: bool):
        self.car_view.set_data_available(ok)
        self._refresh_hud()

    def _on_vid(self, ok: bool):
        self._refresh_hud()

    def _on_vdiag(self, d: dict):
        self.last_diag.update(d)
        self._refresh_hud()

    def _on_sensors(self, d: dict):
        self.last_sensors.update(d)
        self.car_view.update_sensors(self.last_sensors)
        self._refresh_hud()

    def _on_status(self, status_msg: str):
        self.lbl_arduino.setText(f"Arduino: {status_msg}")
        if "timeout" in status_msg.lower():
            self.lbl_arduino.setObjectName("emergencyText")
        else:
            self.lbl_arduino.setObjectName("diagText")
        self.lbl_arduino.setStyle(self.lbl_arduino.style())

    def _on_emergency_stop(self, is_emergency: bool):
        self.emergency_stop = is_emergency
        if is_emergency:
            self.pb_speed.setObjectName("barEmergency")
            self.pb_angle.setObjectName("barEmergency")
            self.lbl_cmd.setObjectName("emergencyText")
            self.lbl_cmd.setText("🚨 EMERGENCY STOP - OBSTACLE DETECTED!")
        else:
            self.pb_speed.setObjectName("barPrimary")
            self.pb_angle.setObjectName("barSecondary")
            self.lbl_cmd.setObjectName("cmdText")

        self.pb_speed.setStyle(self.pb_speed.style())
        self.pb_angle.setStyle(self.pb_angle.style())
        self.lbl_cmd.setStyle(self.lbl_cmd.style())

    def _on_yolo_ready(self, ok: bool):
        self.yolo_ready = ok
        self._refresh_hud()

    def _on_joy(self, j: dict):
        self.last_joy = j
        self.pb_speed.setValue(abs(j['speed']))
        self.pb_angle.setValue(j['angle'] - 40)
        self.lbl_speed.setText(f"Speed: {j['speed']:+4d} (-255..+255)")
        self.lbl_angle.setText(f"Angle: {j['angle']:03d}° (40..140)")

        if self.mode == "MANUAL" and not self.emergency_stop:
            self.mqtt_service.publish_move(j['speed'], j['angle'])
        elif self.emergency_stop:
            self.mqtt_service.publish_move(0, 90)

        self._refresh_hud()

    def _on_conf(self, v: int):
        self.yolo.cfg.conf = v / 100.0
        self._refresh_hud()

    def _on_imgsz(self, v: int):
        self.yolo.cfg.imgsz = int(v)
        self._refresh_hud()

    # ===== IMAGE CAPTURE HANDLERS =====
    def _on_capture_enabled(self, enabled: bool):
        self.image_capture.enabled = enabled
        if hasattr(self, 'capture_button'):
            self.capture_button.setEnabled(enabled)
        if hasattr(self, 'save_button'):
            self.save_button.setEnabled(enabled)
        if not enabled and self.image_capture.capturing:
            self.image_capture.stop_capture()

    def _toggle_capture(self):
        if self.image_capture.capturing:
            self.image_capture.stop_capture()
        else:
            self.image_capture.start_capture()

    def _manual_save(self):
        self.image_capture.save_single_frame("manual")

    def _start_capture_delayed(self):
        if self.image_capture.enabled:
            self.image_capture.start_capture()

    def _on_image_saved(self, filename: str):
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText(f"Saved: {filename[:10]}...")
            QTimer.singleShot(2000, lambda: self.capture_status_label.setText("Ready"))

    def _on_capture_status_changed(self, capturing: bool):
        if hasattr(self, 'capture_button'):
            self.capture_button.setText("Stop" if capturing else "Start")
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText("Recording..." if capturing else "Ready")

    def _on_capture_error(self, error_msg: str):
        print(f"❌ Image capture error: {error_msg}")
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText("Error!")
            QTimer.singleShot(3000, lambda: self.capture_status_label.setText("Ready"))

    # ===== UI UPDATES =====
    def _refresh_hud(self):
        mqtt_ok = self.mqtt_service.is_connected
        video_live = self.last_diag.get('fps', 0) > 0
        sensor_age = self.mqtt_service.get_sensor_age()
        sensor_live = self.mqtt_service.is_sensor_live()
        obstacles = self.mqtt_service.get_obstacles()

        self.lbl_status.setText(f"Mode: {self.mode}")
        self.lbl_video.setText(f"Video: {'LIVE' if video_live else 'OFFLINE'} | FPS {self.last_diag.get('fps', 0):.1f} | seq_err {self.last_diag.get('seq_err', 0)}")

        mqtt_status = "CONNECTED" if mqtt_ok else "OFFLINE"
        if mqtt_ok:
            sensor_status = f"| Sensors: {'LIVE' if sensor_live else 'STALE'} ({sensor_age:.1f}s ago)"
            if obstacles:
                mqtt_status += f" | 🚨 OBSTACLES: {', '.join(obstacles)}"
            else:
                mqtt_status += sensor_status
        self.lbl_mqtt.setText(f"MQTT: {mqtt_status}")

        self.lbl_yolo.setText(f"YOLO: {'READY' if self.yolo_ready else 'IDLE'} | conf {self.yolo.cfg.conf:.2f} | imgsz {self.yolo.cfg.imgsz}")

        if hasattr(self, 'lbl_capture'):
            capture_status = self.image_capture.get_status()
            status_text = f"Capture: {'ON' if capture_status['capturing'] else 'OFF'} | {capture_status['images_saved']} imgs"
            if capture_status['capturing']:
                status_text += f" | {capture_status['save_rate']:.1f}/s"
            self.lbl_capture.setText(status_text)

        if self.emergency_stop:
            self.lbl_cmd.setText("🚨 EMERGENCY STOP - OBSTACLE DETECTED!")
        elif self.mode == "MANUAL":
            self.lbl_cmd.setText(f"CMD(MANUAL): speed={self.last_joy['speed']} angle={self.last_joy['angle']}")
        elif self.mode == "FREE ROAM":
            self.lbl_cmd.setText("CMD(FREE ROAM): sensor-based navigation (no YOLO)")
        else:
            self.lbl_cmd.setText('CMD(AUTONOMOUS): Lane following active')

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        pa = frame.copy()
        cv2.rectangle(pa, (10, 10), (int(w * 0.72), 90), (0, 0, 0), -1)
        blend = cv2.addWeighted(pa, 0.30, frame, 0.70, 0)

        sensor_age = self.mqtt_service.get_sensor_age()
        t1 = f"MODE:{self.mode}   FPS:{self.last_diag.get('fps', 0):.1f}   SENSOR_AGE:{sensor_age:.1f}s"

        s = self.last_sensors
        t2 = f"F:{s.get('vl53', 0):3d}  FL:{s.get('hcsr04_1', 0):3d}  FR:{s.get('hcsr04_2', 0):3d}  B:{s.get('hcsr04_3', 0):3d}"

        color1 = (255, 255, 255)
        color2 = (0, 255, 200) if not self.emergency_stop else (0, 0, 255)

        cv2.putText(blend, t1, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color1, 2)
        cv2.putText(blend, t2, (24, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color2, 2)

        if self.emergency_stop:
            cv2.putText(blend, "🚨 EMERGENCY STOP", (int(w * 0.75), 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        return blend

    def _apply_dark(self):
        self.setStyle(QStyleFactory.create('Fusion'))
        pal = QPalette()
        pal.setColor(QPalette.Window, QColor(16, 16, 18))
        pal.setColor(QPalette.WindowText, QColor(230, 230, 235))
        pal.setColor(QPalette.Base, QColor(12, 12, 14))
        pal.setColor(QPalette.AlternateBase, QColor(24, 24, 26))
        pal.setColor(QPalette.Button, QColor(26, 26, 30))
        pal.setColor(QPalette.ButtonText, QColor(235, 235, 240))
        pal.setColor(QPalette.Text, QColor(240, 240, 245))
        pal.setColor(QPalette.Highlight, QColor(0, 180, 255))
        pal.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        self.setPalette(pal)

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
        QLabel#captureStatus { font-family: "Consolas"; color:#FFA500; font-size: 10px; }
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
        QPushButton { background:#2A2E39; border:1px solid #4A4E59; border-radius:4px; padding:4px 8px; color:#E6E6EA; }
        QPushButton:hover { background:#3A3E49; }
        QPushButton:pressed { background:#1A1E29; }
        QCheckBox { color:#E6E6EA; }
        QCheckBox::indicator:checked { background:#00B4FF; border:1px solid #2A2E39; }
        """)

    def closeEvent(self, event):
        """Clean shutdown"""
        # Stop lane processing
        self.lane_processing = False
        if hasattr(self, 'lane_timer'):
            self.lane_timer.stop()
            
        # Stop image capture
        if hasattr(self, 'image_capture'):
            self.image_capture.stop_capture()

        # Stop services
        if hasattr(self, 'mqtt_service'):
            self.mqtt_service.stop()
        if hasattr(self, 'rtsp'):
            self.rtsp.stop()

        event.accept()