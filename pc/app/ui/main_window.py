from __future__ import annotations
import time, math, numpy as np, cv2
import threading, queue
from dataclasses import dataclass
from collections import deque

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
    AUTO_SPEED, PID_KP, PID_KI, PID_KD, AUTO_CMD_HZ
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
    MODES = ["MANUAL", "AUTONOMOUS"]

    # ===== Planner tuning (single place) =====
    # Sign gating
    MIN_SIGN_CONF = 0.5
    STOP_AREA_MIN = 0.02     # stop signs are roadside and smaller
    ARROW_AREA_MIN = 0.06

    # Sign action thresholds (area fraction to trigger command)
    STOP_AREA_NEAR  = 0.08
    ARROW_AREA_NEAR = 0.06

    # Turn routine
    ARROW_TURN_DURATION_S = 1.1
    ARROW_TURN_SPEED = max(90, min(180, AUTO_SPEED))
    TURN_LEFT_ANGLE  = ANGLE_MIN
    TURN_RIGHT_ANGLE = ANGLE_MAX
    ALIGN_ERR_MAX    = 0.14   # end turn when lane error small
    ALIGN_ANGLE_MAX  = 14.0   # deg
    ALIGN_FRAMES     = 3

    # Stop routine
    STOP_HOLD_SEC = 2

    # Lane steering smoothing / slew
    ANGLE_SMOOTH_ALPHA = 0.4
    ANGLE_MAX_STEP_DEG = 9
    MAX_DEG_PER_SEC    = 160.0

    # Obstacle reasoning relative to lane
    CENTER_BAND = 0.25   
    BIAS_STEP   = 0.22   
    BIAS_DECAY  = 0.95   
    # Stuck handling
    STUCK_FRONT_CM   = 6
    STUCK_TIMEOUT_S  = 0.8
    REAR_SAFE_CM_MIN = 25
    REVERSE_SPEED    = -90
    REVERSE_TIME_S   = 0.7
    THINK_PAUSE_S    = 0.5    # short hold when uncertain

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
        self._last_force = False

        # Perception
        self.det_hist = deque(maxlen=12)      # ~1.5 s at 8 Hz
        self._yolo_hz = 8.0
        self._yolo_pending = False
        self._yolo_last_t = 0.0
        self.detect_in_any_mode = False

        # Planner
        self.auto_last_cmd_t = 0.0
        self.auto_dt = 1.0 / float(AUTO_CMD_HZ)
        self._last_cmd_angle = CENTER_ANGLE
        self.lane_bias = 0.0
        self.alignment_streak = 0
        self.auto_state = "IDLE"  # IDLE | STOP_HOLD | TURN_LEFT | TURN_RIGHT | REVERSING
        self.auto_state_until = 0.0
        self._stuck_since = None
        self._emergency_block_start = None

        # Lane processing
        self.lane_queue = queue.Queue(maxsize=2)
        self.lane_result_queue = queue.Queue(maxsize=5)
        self.lane_processing = False
        self.lane_thread = None
        self.latest_lane_result = LaneResult(error=0.0, debug_info={}, timestamp=0.0)

        self.pid = PID(PID_KP, PID_KI, PID_KD, out_min=-1.0, out_max=1.0)
        self.lane = LaneFollower(
            roi_top_frac=0.65, theta_threshold=2.5, min_line_length=15, max_line_gap=25
        )
        self.lane.set_white_hsv_range([0, 0, 200], [180, 30, 255])
        self.lane.set_canny_thresholds(40, 120)
        self.lane.tune_for_speed()
        self.show_lane_mask = False

        self.lane_timer = QTimer()
        self.lane_timer.timeout.connect(self._process_lane_commands)

        # Services
        self.mqtt_service = MqttService(mcfg)
        self.mqtt_service.start()
        self.mqtt_service.sensorsUpdated.connect(self._on_sensors)
        self.mqtt_service.statusUpdated.connect(self._on_status)
        self.mqtt_service.connectedChanged.connect(self._on_mqtt)
        self.mqtt_service.emergencyStopChanged.connect(self._on_emergency_stop)

        self.last_safety = None
        if hasattr(self.mqtt_service, "safetyUpdated"):
            self.mqtt_service.safetyUpdated.connect(self._on_safety)

        self.joy = JoystickService()
        self.joy.joystickChanged.connect(self._on_joy)
        if hasattr(self.joy, "captureImageRequested"):
            self.joy.captureImageRequested.connect(self._on_joystick_capture)

        self.rtsp = RtspWorker(vcfg.rtsp_url, rtp_port=vcfg.rtp_port, flip_vertical=vcfg.flip_vertical)
        self.rtsp.frameReady.connect(self._on_frame)
        self.rtsp.diag.connect(self._on_vdiag)
        self.rtsp.connectedChanged.connect(self._on_vid)
        self.rtsp.start()

        self.yolo = YoloWorker(ycfg)
        self.yolo.modelReady.connect(self._on_yolo_ready)
        self.yolo.annotatedReady.connect(self._on_annotated)

        self.image_capture = ImageCaptureService(
            save_directory=icfg.save_directory, interval_ms=icfg.interval_ms
        )
        self.image_capture.enabled = icfg.enabled
        self.image_capture.imageSaved.connect(self._on_image_saved)
        self.image_capture.captureStatusChanged.connect(self._on_capture_status_changed)
        self.image_capture.errorOccurred.connect(self._on_capture_error)

        # ==== UI ======================================================
        topbar = QHBoxLayout()
        title = QLabel('DASHBOARD'); title.setObjectName("appTitle")
        topbar.addWidget(title); topbar.addStretch(1)

        topbar.addWidget(QLabel('MODE')); topbar.addSpacing(8)
        
        mode_container = QWidget()
        mode_layout = QHBoxLayout(mode_container)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(12)  # Spacing between labels
        
        # Clickable labels
        self.manual_label = QLabel("MANUAL")
        self.manual_label.setObjectName("modeLabelActive")  # Start active
        self.manual_label.mousePressEvent = lambda event: self._on_manual_click()
        self.manual_label.setCursor(Qt.PointingHandCursor)
        
        self.autonomous_label = QLabel("AUTONOMOUS")
        self.autonomous_label.setObjectName("modeLabelInactive")  # Start inactive
        self.autonomous_label.mousePressEvent = lambda event: self._on_autonomous_click()
        self.autonomous_label.setCursor(Qt.PointingHandCursor)
        
        mode_layout.addWidget(self.manual_label)
        mode_layout.addWidget(self.autonomous_label)
        
        topbar.addWidget(mode_container); topbar.addSpacing(16)


        self.sld_conf = QSlider(Qt.Horizontal); self.sld_conf.setRange(5, 90)
        self.sld_conf.setValue(int(ycfg.conf * 100)); self.sld_conf.valueChanged.connect(self._on_conf)
        self.sp_imgsz = QSpinBox(); self.sp_imgsz.setRange(320, 1280); self.sp_imgsz.setSingleStep(64)
        self.sp_imgsz.setValue(ycfg.imgsz); self.sp_imgsz.valueChanged.connect(self._on_imgsz)
        topbar.addWidget(QLabel('YOLO CONF')); topbar.addWidget(self.sld_conf)
        topbar.addWidget(QLabel('IMGSZ')); topbar.addWidget(self.sp_imgsz)

        topbar.addSpacing(12)
        self.cb_detect = QCheckBox("Detections"); self.cb_detect.setChecked(False)
        self.cb_detect.toggled.connect(self._on_detect_toggle)
        topbar.addWidget(self.cb_detect)


        self.manual_label.setObjectName("modeLabelActive")  # Start with MANUAL active
        self.autonomous_label.setObjectName("modeLabelInactive")
        # Left column
        self.video_label = QLabel('VIDEO')
        self.video_label.setFrameStyle(QFrame.StyledPanel)
        self.video_label.setMinimumSize(520, 292)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.car_view = CarSensorView()

        left_col = QVBoxLayout()
        left_col.addWidget(self.video_label, 1)
        left_col.addWidget(self.car_view, 1)

        # Right column
        self.grp_diag = QGroupBox('DIAGNOSTICS')
        diag_lay = QVBoxLayout(self.grp_diag)
        self.lbl_status = QLabel('Status: …')
        self.lbl_video  = QLabel('Video: …')
        self.lbl_mqtt   = QLabel('MQTT: …')
        self.lbl_yolo   = QLabel('YOLO: …')
        self.lbl_diag   = QLabel('Perf: …')
        self.lbl_arduino= QLabel('Arduino: …')
        for w in (self.lbl_status, self.lbl_video, self.lbl_mqtt, self.lbl_yolo, self.lbl_diag, self.lbl_arduino):
            w.setObjectName("diagText")
            diag_lay.addWidget(w)

        # Perception panel
        self.grp_perc = QGroupBox('PERCEPTION')
        perc_lay = QVBoxLayout(self.grp_perc)
        self.lbl_detected = QLabel('Detected: -'); self.lbl_detected.setObjectName("diagText")
        self.lbl_action   = QLabel('Action: -');   self.lbl_action.setObjectName("diagText")
        self.pb_action    = QProgressBar(); self.pb_action.setRange(0, 100); self.pb_action.setValue(0); self.pb_action.setVisible(False)
        perc_lay.addWidget(self.lbl_detected); perc_lay.addWidget(self.lbl_action); perc_lay.addWidget(self.pb_action)

        # HUD
        self.grp_hud = QGroupBox('DRIVE HUD')
        hud_lay = QGridLayout(self.grp_hud)
        self.lbl_speed = QLabel('Speed: +000 (-255..+255)')
        self.pb_speed = QProgressBar(); self.pb_speed.setRange(0, 255); self.pb_speed.setObjectName("barPrimary")
        self.lbl_angle = QLabel('Angle: 090° (40..140)')
        self.pb_angle = QProgressBar(); self.pb_angle.setRange(0, 100); self.pb_angle.setObjectName("barSecondary")
        self.lbl_cmd = QLabel('CMD: …'); self.lbl_cmd.setObjectName("cmdText")
        hud_lay.addWidget(self.lbl_speed, 0, 0); hud_lay.addWidget(self.pb_speed, 0, 1)
        hud_lay.addWidget(self.lbl_angle, 1, 0); hud_lay.addWidget(self.pb_angle, 1, 1)
        hud_lay.addWidget(self.lbl_cmd, 2, 0, 1, 2)

        right_col = QVBoxLayout()
        right_col.addWidget(self.grp_diag, 2)
        right_col.addWidget(self.grp_perc, 1)
        right_col.addWidget(self.grp_hud, 1)
        right_col.addStretch(1)

        root = QVBoxLayout(self)
        root.addLayout(topbar)
        body = QHBoxLayout()
        body.addLayout(left_col, 1)
        body.addLayout(right_col, 1)
        root.addLayout(body, 1)

        # Timers
        self.hud_timer = QTimer(self)
        self.hud_timer.setInterval(200)
        self.hud_timer.timeout.connect(self._refresh_hud)
        self.hud_timer.start()

        self.car_view.set_data_available(False)
        if icfg.enabled and icfg.auto_start:
            QTimer.singleShot(3000, self._start_capture_delayed)

        # finalize YOLO status
        self._on_yolo_ready(bool(getattr(self.yolo, "model", None)))

    # ===== Frame path =====
    def _on_frame(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        self.last_frame_size = (w, h)
        self.current_frame = frame_bgr.copy()

        if hasattr(self, 'image_capture'):
            self.image_capture.update_original_frame(frame_bgr)

        overlay_frame = self._draw_overlay(frame_bgr)
        if hasattr(self, 'image_capture'):
            self.image_capture.update_overlay_frame(overlay_frame)

        img = self.bgr_to_qimage(overlay_frame)
        self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        # YOLO scheduling
        self._maybe_submit_yolo(frame_bgr)

        # Lane scheduling
        if self.mode == "AUTONOMOUS":
            self._queue_frame_for_lane_processing(frame_bgr)
        elif self.mode == "FREE ROAM" and not self.emergency_stop:
            decision = decide_command([], w, h, self.last_sensors)
            self.lbl_cmd.setText(f"FREE ROAM decision: steer {decision['steer']:+.2f} throttle {decision['throttle']:+.2f} ({decision['action']})")

    def _queue_frame_for_lane_processing(self, frame_bgr):
        try:
            if not self.lane_queue.full():
                h, w = frame_bgr.shape[:2]
                small = cv2.resize(frame_bgr, (w//2, h//2))
                self.lane_queue.put_nowait(small)
                if not self.lane_processing:
                    self._start_lane_processing_thread()
        except queue.Full:
            pass

    def _start_lane_processing_thread(self):
        if self.lane_thread is None or not self.lane_thread.is_alive():
            self.lane_processing = True
            self.lane_thread = threading.Thread(target=self._lane_processing_worker, daemon=True)
            self.lane_thread.start()

    def _lane_processing_worker(self):
        while self.lane_processing and self.mode == "AUTONOMOUS":
            try:
                frame = self.lane_queue.get(timeout=0.1)
                start = time.time()
                lane_error, debug_frame = self.lane.compute_error(frame)
                proc_ms = (time.time() - start) * 1000
                if self.show_lane_mask and debug_frame is not None:
                    cv2.imshow("White Lane Detection", debug_frame); cv2.waitKey(1)
                info = self.lane.get_debug_info()
                info['processing_time_ms'] = proc_ms
                result = LaneResult(error=lane_error, debug_info=info, timestamp=time.time())
                if not self.lane_result_queue.full():
                    self.lane_result_queue.put_nowait(result)
                self.lane_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                pass
        self.lane_processing = False

    # ===== Autonomous loop =====
    def _process_lane_commands(self):
        if self.mode != "AUTONOMOUS":
            return
        now = time.time()

        # pull latest lane result
        while not self.lane_result_queue.empty():
            self.latest_lane_result = self.lane_result_queue.get_nowait()

        # lane metrics
        res_age = now - self.latest_lane_result.timestamp
        if res_age > 0.25:
            lane_error = 0.0
            lane_metrics = {'confidence': 0.0, 'angle_deg': 0.0, 'center_x_norm': 0.5, 'roi_top_frac': 0.65}
            debug_info = {'detected_lines': 0, 'last_theta': 0.0, 'processing_time_ms': 0}
        else:
            lane_error = self.latest_lane_result.error
            lane_metrics = self.lane.get_lane_metrics()
            debug_info = self.latest_lane_result.debug_info

        # rate limit
        if now - self.auto_last_cmd_t < self.auto_dt:
            return
        dt = now - self.auto_last_cmd_t if self.auto_last_cmd_t > 0 else self.auto_dt

        # detection stability
        stop_seen  = self._seen_frames("stop",      min_frames=6)
        left_seen  = self._seen_frames("left",      min_frames=5)
        right_seen = self._seen_frames("right",     min_frames=5)
        obs_left   = self._seen_frames("obs_left",  min_frames=4)
        obs_right  = self._seen_frames("obs_right", min_frames=4)

        # start actions from signs (priority)
        if self.auto_state == "IDLE":
            if stop_seen:
                self.auto_state = "STOP_HOLD"
                self.auto_state_until = now + self.STOP_HOLD_SEC
                self.lbl_action.setText("Action: Stop")
                self.pb_action.setVisible(True); self.pb_action.setValue(0)
            elif left_seen:
                self.auto_state = "TURN_LEFT"
                self.auto_state_until = now + self.ARROW_TURN_DURATION_S
                self.lbl_action.setText("Action: Turn left")
                self.pb_action.setVisible(True); self.pb_action.setValue(0)
            elif right_seen:
                self.auto_state = "TURN_RIGHT"
                self.auto_state_until = now + self.ARROW_TURN_DURATION_S
                self.lbl_action.setText("Action: Turn right")
                self.pb_action.setVisible(True); self.pb_action.setValue(0)

        # obstacle influence (only when not executing sign action)
        if self.auto_state == "IDLE":
            if obs_left and not obs_right:
                self.lane_bias = +self.BIAS_STEP  # push right if obstacle is left of lane
                self.lbl_action.setText("Action: Avoid left-side obstacle")
            elif obs_right and not obs_left:
                self.lane_bias = -self.BIAS_STEP  # push left if obstacle is right of lane
                self.lbl_action.setText("Action: Avoid right-side obstacle")
            elif obs_left and obs_right:
                # both sides show obstacles on-lane → slight bias to the side with more clearance by sensors
                left_clear = self.last_sensors.get('hcsr04_1', 0) > 15
                right_clear = self.last_sensors.get('hcsr04_2', 0) > 15
                if left_clear and not right_clear:
                    self.lane_bias = -self.BIAS_STEP * 0.7
                    self.lbl_action.setText("Action: Avoid right using left clearance")
                elif right_clear and not left_clear:
                    self.lane_bias = +self.BIAS_STEP * 0.7
                    self.lbl_action.setText("Action: Avoid left using right clearance")

        # execute state machines
        if self.auto_state == "STOP_HOLD":
            # full stop
            if not self.emergency_stop:
                self.mqtt_service.publish_move(0, CENTER_ANGLE)
                self.auto_last_cmd_t = now
            # finish
            if now >= self.auto_state_until:
                self.auto_state = "IDLE"
                self.pb_action.setVisible(False)
                self.lbl_action.setText("Action: Lane follow")
            else:
                self._update_action_progress(self.auto_state_until, now, "Stop")
            self.lbl_cmd.setText("AUTONOMOUS: Stop")
            return

        # eMERGENCY REVERSE STATE HANDLEr
        if self.auto_state == "EMERGENCY_REVERSE":
            # Emergency reverse for 1 second
            if not self.emergency_stop:
                self.mqtt_service.publish_move(self.REVERSE_SPEED, CENTER_ANGLE)
                self.auto_last_cmd_t = now
            if now >= self.auto_state_until:
                self.auto_state = "IDLE"  # Return to normal operation
                self.pb_action.setVisible(False)
                self.lbl_action.setText("Action: Lane follow")
            else:
                self._update_action_progress(self.auto_state_until, now, "Emergency Reverse")
            self.lbl_cmd.setText("AUTONOMOUS: Emergency Reverse")
            return

        if self.auto_state in ("TURN_LEFT", "TURN_RIGHT"):
            turning_left = (self.auto_state == "TURN_LEFT")
            turn_angle = self.TURN_LEFT_ANGLE if turning_left else self.TURN_RIGHT_ANGLE
            elapsed = (self.ARROW_TURN_DURATION_S - max(0.0, self.auto_state_until - now))
            progress = max(0.0, min(1.0, elapsed / self.ARROW_TURN_DURATION_S))

            if progress < 0.7:
                target_angle = turn_angle
                turn_speed = self.ARROW_TURN_SPEED
            else:
                # blend in lane correction
                biased_error = np.clip(lane_error + self.lane_bias, -1.0, 1.0)
                steering_output = self.pid.compute(biased_error, dt)
                lane_angle = CENTER_ANGLE + int(steering_output * ANGLE_RANGE)
                lane_angle = max(ANGLE_MIN, min(ANGLE_MAX, lane_angle))
                blend = (progress - 0.7) / 0.3  # 0..1 over last 30%
                target_angle = int(turn_angle * (1.0 - blend) + lane_angle * blend)
                target_angle = max(ANGLE_MIN, min(ANGLE_MAX, target_angle))
                turn_speed = max(60, int(AUTO_SPEED * 0.7))

            # end early if well aligned
            if (progress > 0.5 and
                lane_metrics['confidence'] > 0.6 and
                abs(lane_error) <= self.ALIGN_ERR_MAX and
                abs(lane_metrics['angle_deg']) <= self.ALIGN_ANGLE_MAX):
                self.alignment_streak += 1
            else:
                self.alignment_streak = 0

            if self.alignment_streak >= self.ALIGN_FRAMES or now >= self.auto_state_until:
                self.auto_state = "IDLE"
                self.alignment_streak = 0
                self.lane_bias = 0.0
                self.pb_action.setVisible(False)
                self.lbl_action.setText("Action: Lane follow")
            else:
                if not self.emergency_stop:
                    self.mqtt_service.publish_move(turn_speed, target_angle)
                    self.auto_last_cmd_t = now
                self._update_action_progress(self.auto_state_until, now, "Turn")
                self.lbl_cmd.setText(f"AUTONOMOUS: {'Left' if turning_left else 'Right'} turn")
                return

        # Uncertain? Short pause to stabilize
        if self.auto_state == "IDLE":
            if lane_metrics['confidence'] < 0.25 and not (left_seen or right_seen or stop_seen):
                if not self.emergency_stop:
                    self.mqtt_service.publish_move(0, CENTER_ANGLE)
                    self.auto_last_cmd_t = now
                self.lbl_action.setText("Action: Pause (low lane confidence)")
                self.lbl_cmd.setText("AUTONOMOUS: Pause")
                self.auto_state = "STOP_HOLD"
                self.auto_state_until = now + self.THINK_PAUSE_S
                self.pb_action.setVisible(True); self.pb_action.setValue(0)
                return

        # Stuck detection and recovery (only when IDLE)
        if self.auto_state == "IDLE":
            front = self.last_sensors.get('vl53') or 0
            if 0 < front <= self.STUCK_FRONT_CM:
                if self._stuck_since is None:
                    self._stuck_since = now
                elif now - self._stuck_since >= self.STUCK_TIMEOUT_S:
                    rear = self.last_sensors.get('hcsr04_3') or 0
                    if rear >= self.REAR_SAFE_CM_MIN:
                        self.auto_state = "REVERSING"
                        self.auto_state_until = now + self.REVERSE_TIME_S
                        self.lbl_action.setText("Action: Reverse to free space")
                        self.pb_action.setVisible(True); self.pb_action.setValue(0)
                    else:
                        self.auto_state = "STOP_HOLD"
                        self.auto_state_until = now + self.THINK_PAUSE_S
                        self.lbl_action.setText("Action: Blocked")
                        self.pb_action.setVisible(True); self.pb_action.setValue(0)
                    self._stuck_since = None
                    return
            else:
                self._stuck_since = None

        if self.auto_state == "REVERSING":
            if not self.emergency_stop:
                self.mqtt_service.publish_move(self.REVERSE_SPEED, CENTER_ANGLE)
                self.auto_last_cmd_t = now
            if now >= self.auto_state_until:
                self.auto_state = "IDLE"
                self.pb_action.setVisible(False)
                self.lbl_action.setText("Action: Lane follow")
            else:
                self._update_action_progress(self.auto_state_until, now, "Reverse")
            self.lbl_cmd.setText("AUTONOMOUS: Reverse")
            return

        # Normal lane following (IDLE)
        if abs(self.lane_bias) > 0.01:
            self.lane_bias *= self.BIAS_DECAY
            if abs(self.lane_bias) < 0.01:
                self.lane_bias = 0.0

        biased_error = np.clip(lane_error + self.lane_bias, -1.0, 1.0)
        steering_output = self.pid.compute(biased_error, dt)
        angle = CENTER_ANGLE + int(steering_output * ANGLE_RANGE)
        angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))

        max_step = self.MAX_DEG_PER_SEC * dt
        delta = float(angle - self._last_cmd_angle)
        if abs(delta) > max_step:
            angle = int(self._last_cmd_angle + np.sign(delta) * max_step)
        self._last_cmd_angle = angle

        # EMERGENCY BRAKING LOGIC 
        front = self.last_sensors.get('vl53') or 0
        left = self.last_sensors.get('hcsr04_1') or 0  
        right = self.last_sensors.get('hcsr04_2') or 0
        
        # HARD BLOCK - no forward movement allowed
        emergency_block = False
        if 0 < front < 5:  # Front sensor critical
            emergency_block = True
        elif 0 < left < 3:  # Left sensor critical
            emergency_block = True
        elif 0 < right < 3:  # Right sensor critical  
            emergency_block = True
            
        # If blocked, check for sustained emergency (3+ seconds)
        if emergency_block:
            if self._emergency_block_start is None:
                self._emergency_block_start = now
            elif (now - self._emergency_block_start) >= 3.0:  # 3 seconds sustained
                # Check if we can reverse safely
                rear = self.last_sensors.get('hcsr04_3') or 0
                if rear >= self.REAR_SAFE_CM_MIN and self.auto_state != "EMERGENCY_REVERSE":
                    self.auto_state = "EMERGENCY_REVERSE"
                    self.auto_state_until = now + 1.0  # Reverse for 1 second
                    self._emergency_block_start = None  # Reset timer
                    self.lbl_action.setText("Action: Emergency reverse (3s blocked)")
                    self.pb_action.setVisible(True); self.pb_action.setValue(0)
                    return
                else:
                    # Can't reverse safely - stay stopped
                    throttle = 0
            else:
                # Still in emergency block period
                throttle = 0
        else:
            # Not blocked - reset emergency timer
            self._emergency_block_start = None
            
            # Normal speed control (only if not emergency blocked)
            throttle = AUTO_SPEED
            if 0 < front <= 10:   # Slow down when getting close
                throttle = max(60, int(AUTO_SPEED * 0.7))

        if not self.emergency_stop:
            self.mqtt_service.publish_move(throttle, angle)
            self.auto_last_cmd_t = now

        proc_ms = debug_info.get('processing_time_ms', 0)
        self.lbl_cmd.setText(
            f"AUTONOMOUS: lane err={lane_error:.3f} bias={self.lane_bias:+.2f} conf={lane_metrics['confidence']:.2f} "
            f"angle={angle} speed={throttle} proc={proc_ms:.1f}ms"
        )

    # ===== YOLO callback =====
    def _on_annotated(self, annotated_bgr, dets):
        self._yolo_pending = False

        # Show overlay when checkbox is on
        if self.detect_in_any_mode:
            shown = self._draw_overlay(annotated_bgr)
            img = self.bgr_to_qimage(shown)
            self.video_label.setPixmap(QPixmap.fromImage(img).scaled(
                self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        # Summarize detections for planner
        w, h = self.last_frame_size
        metrics = self.lane.get_lane_metrics()
        lane_cx_norm = float(metrics.get('center_x_norm', 0.5))
        roi_top = float(metrics.get('roi_top_frac', 0.65))
        lane_cx_px = lane_cx_norm * w
        band_px = self.CENTER_BAND * w  # band width around lane center

        frame_summary = {"stop": 0, "left": 0, "right": 0, "obs_left": 0, "obs_right": 0}
        detected_names = []
        best_obs_area = 0.0
        best_obs_side = None

        for name, conf, x1, y1, x2, y2 in dets:
            name_l = name.lower()
            if conf < self.MIN_SIGN_CONF:
                continue

            area = self._box_area_ratio(x1, y1, x2, y2, w, h)
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)

            if name_l == "stop" and area >= self.STOP_AREA_MIN:
                frame_summary["stop"] = 1
                detected_names.append(f"STOP {area*100:.1f}%@{conf:.2f}")

            elif name_l in ("arrow_left", "left") and area >= self.ARROW_AREA_MIN:
                frame_summary["left"] = 1
                detected_names.append(f"LEFT {area*100:.1f}%@{conf:.2f}")

            elif name_l in ("arrow_right", "right") and area >= self.ARROW_AREA_MIN:
                frame_summary["right"] = 1
                detected_names.append(f"RIGHT {area*100:.1f}%@{conf:.2f}")

            elif name_l == "obstacle":
                if cy >= roi_top * h:
                    if abs(cx - lane_cx_px) <= band_px:
                        if area > best_obs_area:
                            best_obs_area = area
                            best_obs_side = "left" if cx < lane_cx_px else "right"

        if best_obs_side == "left":
            frame_summary["obs_left"] = 1
            detected_names.append("OBS lane-left")
        elif best_obs_side == "right":
            frame_summary["obs_right"] = 1
            detected_names.append("OBS lane-right")

        self.det_hist.append(frame_summary)
        self.lbl_detected.setText(f"Detected: {', '.join(detected_names) if detected_names else '-'}")

    # ===== Small helpers =====
    def _on_detect_toggle(self, on: bool):
        self.detect_in_any_mode = on
        if not on:
            self._yolo_pending = False

    def _should_run_yolo(self) -> bool:
        return self.mode == "AUTONOMOUS" or self.detect_in_any_mode

    def _maybe_submit_yolo(self, frame_bgr):
        if not self._should_run_yolo():
            return
        if not (self.yolo_ready or getattr(self.yolo, "model", None)):
            return
        now = time.time()
        if self._yolo_pending or (now - self._yolo_last_t) < (1.0 / self._yolo_hz):
            return
        self._yolo_pending = True
        self._yolo_last_t = now
        if hasattr(self.yolo, "infer_async"):
            self.yolo.infer_async(frame_bgr.copy())
        else:
            self.yolo.infer(frame_bgr.copy())

    def _seen_frames(self, key: str, min_frames: int) -> bool:
        if not self.det_hist:
            return False
        return sum(1 for f in self.det_hist if f.get(key, 0) > 0) >= int(min_frames)

    @staticmethod
    def _box_area_ratio(x1, y1, x2, y2, w, h):
        bw = max(0.0, x2 - x1); bh = max(0.0, y2 - y1)
        return (bw * bh) / float(max(1.0, w * h))

    def _update_action_progress(self, t_until: float, now: float, label: str):
        span = max(0.01, t_until - (now - 1e-3))
        rem = max(0.0, t_until - now)
        pct = int(100.0 * (1.0 - (rem / max(0.001, span))))
        self.pb_action.setVisible(True)
        self.pb_action.setValue(max(0, min(100, pct)))
        self.lbl_action.setText(f"Action: {label} ({rem:.2f}s left)")

    # ===== UI & system glue =====
    def _on_joystick_capture(self):
        if hasattr(self, 'image_capture'):
            self.image_capture.capture_from_joystick()

    def bgr_to_qimage(self, bgr_frame):
        try:
            rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape; bpl = ch * w
            return QImage(rgb.data, w, h, bpl, QImage.Format_RGB888)
        except Exception:
            black = np.zeros((480, 640, 3), dtype=np.uint8)
            rgb = cv2.cvtColor(black, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape; bpl = ch * w
            return QImage(rgb.data, w, h, bpl, QImage.Format_RGB888)

    def _on_mode_changed(self, new_mode: str):
        self.mode = new_mode
        if new_mode == "AUTONOMOUS":
            self.lane_timer.start(int(1000 / AUTO_CMD_HZ))
            self.pid.reset()
        else:
            self.lane_timer.stop()
            self.lane_processing = False
            while not self.lane_queue.empty():
                try: self.lane_queue.get_nowait()
                except queue.Empty: break
            while not self.lane_result_queue.empty():
                try: self.lane_result_queue.get_nowait()
                except queue.Empty: break

        self.auto_state = "IDLE"
        self.auto_state_until = 0.0
        self._last_cmd_angle = CENTER_ANGLE
        self.lane_bias = 0.0
        self._stuck_since = None
        self._emergency_block_start = None  
        self._refresh_hud()

    def _on_mqtt(self, ok: bool):
        self.car_view.set_data_available(ok); self._refresh_hud()

    def _on_vid(self, ok: bool):
        self._refresh_hud()

    def _on_vdiag(self, d: dict):
        self.last_diag.update(d); self._refresh_hud()

    def _on_sensors(self, d: dict):
        self.last_sensors.update(d); self.car_view.update_sensors(self.last_sensors); self._refresh_hud()

    def _on_status(self, status_msg: str):
        self.lbl_arduino.setText(f"Arduino: {status_msg}")
        self.lbl_arduino.setObjectName("diagText")
        self.lbl_arduino.setStyle(self.lbl_arduino.style())

    def _on_emergency_stop(self, is_emergency: bool):
        self.emergency_stop = is_emergency
        if is_emergency:
            self.pb_speed.setObjectName("barEmergency")
            self.pb_angle.setObjectName("barEmergency")
            self.lbl_cmd.setObjectName("emergencyText")
            self.lbl_cmd.setText("EMERGENCY STOP - OBSTACLE DETECTED")
        else:
            self.pb_speed.setObjectName("barPrimary")
            self.pb_angle.setObjectName("barSecondary")
            self.lbl_cmd.setObjectName("cmdText")
        self.pb_speed.setStyle(self.pb_speed.style())
        self.pb_angle.setStyle(self.pb_angle.style())
        self.lbl_cmd.setStyle(self.lbl_cmd.style())

    def _on_yolo_ready(self, ok: bool):
        self.yolo_ready = ok; self._refresh_hud()

    def _on_joy(self, j: dict):
        self.last_joy = j
        self.pb_speed.setValue(abs(j['speed']))
        self.pb_angle.setValue(j['angle'] - 40)
        self.lbl_speed.setText(f"Speed: {j['speed']:+4d} (-255..+255)")
        self.lbl_angle.setText(f"Angle: {j['angle']:03d}° (40..140)")
        self._last_force = bool(j.get('force', False))

        if self.mode == "MANUAL" and not self.emergency_stop:
            try:
                self.mqtt_service.publish_move(j['speed'], j['angle'], force_override=self._last_force)
            except TypeError:
                self.mqtt_service.publish_move(j['speed'], j['angle'])
        elif self.emergency_stop:
            self.mqtt_service.publish_move(0, CENTER_ANGLE)
        self._refresh_hud()

    def _on_conf(self, v: int):
        self.yolo.cfg.conf = v / 100.0; self._refresh_hud()

    def _on_imgsz(self, v: int):
        self.yolo.cfg.imgsz = int(v); self._refresh_hud()

    # Image capture
    def _on_capture_enabled(self, enabled: bool):
        self.image_capture.enabled = enabled
        if hasattr(self, 'capture_button'):
            self.capture_button.setEnabled(enabled)
        if hasattr(self, 'save_button'):
            self.save_button.setEnabled(enabled)
        if not enabled and self.image_capture.capturing:
            self.image_capture.stop_capture()

    def _toggle_capture(self):
        if self.image_capture.capturing: self.image_capture.stop_capture()
        else: self.image_capture.start_capture()

    def _manual_save(self):
        self.image_capture.save_single_frame("manual")

    def _start_capture_delayed(self):
        if self.image_capture.enabled: self.image_capture.start_capture()

    def _on_image_saved(self, filename: str):
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText(f"Saved: {filename[:10]}...")

    def _on_capture_status_changed(self, capturing: bool):
        if hasattr(self, 'capture_button'):
            self.capture_button.setText("Stop" if capturing else "Start")
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText("Recording..." if capturing else "Ready")

    def _on_capture_error(self, error_msg: str):
        if hasattr(self, 'capture_status_label'):
            self.capture_status_label.setText("Error!")

    # HUD
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
                mqtt_status += f" | OBSTACLES: {', '.join(obstacles)}"
            else:
                mqtt_status += sensor_status
        self.lbl_mqtt.setText(f"MQTT: {mqtt_status}")

        det_flag = "RUN" if self._should_run_yolo() and self.yolo_ready else "OFF"
        self.lbl_yolo.setText(f"YOLO: {'READY' if self.yolo_ready else 'IDLE'} | conf {self.yolo.cfg.conf:.2f} | imgsz {self.yolo.cfg.imgsz} | DET:{det_flag}")

        if self.emergency_stop:
            self.lbl_cmd.setText("EMERGENCY STOP - OBSTACLE DETECTED")
        elif self.mode == "MANUAL":
            ftxt = " (FORCE)" if self._last_force else ""
            self.lbl_cmd.setText(f"CMD(MANUAL): speed={self.last_joy['speed']} angle={self.last_joy['angle']}{ftxt}")
        elif self.mode == "FREE ROAM":
            self.lbl_cmd.setText("CMD(FREE ROAM): sensor-based navigation")
        else:
            self.lbl_cmd.setText(f"CMD(AUTONOMOUS): {self.auto_state}")

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        pa = frame.copy()
        cv2.rectangle(pa, (10, 10), (int(w * 0.72), 90), (0, 0, 0), -1)
        blend = cv2.addWeighted(pa, 0.30, frame, 0.70, 0)

        sensor_age = self.mqtt_service.get_sensor_age()
        t1 = f"MODE:{self.mode}   FPS:{self.last_diag.get('fps', 0):.1f}   SENSOR_AGE:{sensor_age:.1f}s   FORCE:{'ON' if self._last_force else 'OFF'}"

        s = self.last_sensors
        t2 = f"F:{s.get('vl53', 0):3d}  FL:{s.get('hcsr04_1', 0):3d}  FR:{s.get('hcsr04_2', 0):3d}  B:{s.get('hcsr04_3', 0):3d}"

        color1 = (255, 255, 255)
        color2 = (0, 255, 200) if not self.emergency_stop else (0, 0, 255)

        cv2.putText(blend, t1, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color1, 2)
        cv2.putText(blend, t2, (24, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color2, 2)

        if self.emergency_stop:
            cv2.putText(blend, "EMERGENCY STOP", (int(w * 0.75), 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        return blend



    def _on_manual_click(self):
        """Handle MANUAL label click"""
        if self.mode != "MANUAL":
            # Switch to MANUAL
            self.manual_label.setObjectName("modeLabelActive")
            self.autonomous_label.setObjectName("modeLabelInactive")
            
            # Apply styles
            self.manual_label.setStyle(self.manual_label.style())
            self.autonomous_label.setStyle(self.autonomous_label.style())
            
            # Call existing mode change logic
            self._on_mode_changed("MANUAL")

    def _on_autonomous_click(self):
        """Handle AUTONOMOUS label click"""
        if self.mode != "AUTONOMOUS":
            # Switch to AUTONOMOUS
            self.manual_label.setObjectName("modeLabelInactive")
            self.autonomous_label.setObjectName("modeLabelActive")
            
            # Apply styles
            self.manual_label.setStyle(self.manual_label.style())
            self.autonomous_label.setStyle(self.autonomous_label.style())
            
            self._on_mode_changed("AUTONOMOUS")

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
        
        QLabel#modeLabel {
            font-size: 13px;
            font-weight: 600;
            color: #AAA;
            padding: 6px 12px;
            border-radius: 4px;
        }
        QLabel#modeLabelActive {
            font-size: 13px;
            font-weight: 700;
            color: #FFFFFF;
            padding: 6px 12px;
            background: rgba(0, 102, 204, 0.3);
            border: 1px solid rgba(0, 136, 255, 0.5);
            border-radius: 4px;
        }
        QLabel#modeLabelInactive {
            font-size: 13px;
            font-weight: 500;
            color: #888;
            padding: 6px 12px;
            border-radius: 4px;
            border: 1px solid transparent;
        }
        QLabel#modeLabelInactive:hover {
            background: rgba(58, 62, 73, 0.3);
            color: #AAA;
            border: 1px solid rgba(74, 78, 89, 0.5);
        }
        
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
        self.lane_processing = False
        if hasattr(self, 'lane_timer'): self.lane_timer.stop()
        if hasattr(self, 'image_capture'): self.image_capture.stop_capture()
        if hasattr(self, 'mqtt_service'): self.mqtt_service.stop()
        if hasattr(self, 'rtsp'): self.rtsp.stop()
        event.accept()

    def _on_safety(self, s: dict):
        self.last_safety = s
        self._refresh_hud()
