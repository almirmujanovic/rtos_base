import cv2
import numpy as np
import math
import time

class LaneFollower:
    """
    White-lane follower optimized for bottom-ROI detection with simple quality metrics.
    Exposes normalized lane center so downstream logic can reason about left/right wrt lane.
    """

    def __init__(self, roi_top_frac=0.65, theta_threshold=2.5, min_line_length=15, max_line_gap=25):
        self.roi_top_frac = float(roi_top_frac)
        self.theta_threshold = float(theta_threshold)
        self.min_line_length = int(min_line_length)
        self.max_line_gap = int(max_line_gap)

        # White detection
        self.white_threshold_gray = 180
        self.white_threshold_hsv_low = [0, 0, 200]
        self.white_threshold_hsv_high = [180, 30, 255]

        # Edges / morphology
        self.canny_low = 40
        self.canny_high = 120
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.blur_kernel = (5, 5)

        # State
        self.last_theta = 0.0
        self.detected_lines = 0
        self.last_error = 0.0
        self.last_lane_center_norm = 0.5  # normalized in ROI [0..1]
        self._last_valid = 0.0            # timestamp of last valid detection

        # Smoothing memory
        self.lane_memory = []
        self.max_memory = 5

    def compute_error(self, frame):
        """Return (steering_error [-1..1], debug_frame)."""
        try:
            h, w = frame.shape[:2]

            # ROI
            roi_start = int(self.roi_top_frac * h)
            roi = frame[roi_start:, :]
            roi_h, roi_w = roi.shape[:2]

            # Resize for speed
            target_w = 320
            target_h = int((target_w / roi_w) * roi_h)
            roi_r = cv2.resize(roi, (target_w, target_h))

            # White mask (gray + HSV)
            white_mask = self._detect_white(roi_r)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.morph_kernel)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.morph_kernel)

            # Edges
            edges = cv2.Canny(white_mask, self.canny_low, self.canny_high)

            # Lines
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=8,
                                    minLineLength=self.min_line_length,
                                    maxLineGap=self.max_line_gap)

            # Analyze
            steering_error, lane_info = self._analyze(lines, target_w, target_h)

            # Debug frame (optional)
            debug = self._debug_panel(roi_r, white_mask, edges, lines, lane_info)

            # Record validity
            if lane_info.get('status') == 'TRACKING':
                self._last_valid = time.time()

            return float(steering_error), debug

        except Exception:
            return 0.0, None

    def _detect_white(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur_kernel, 0)
        _, white_gray = cv2.threshold(gray, self.white_threshold_gray, 255, cv2.THRESH_BINARY)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_hsv = cv2.inRange(hsv, np.array(self.white_threshold_hsv_low), np.array(self.white_threshold_hsv_high))

        combined = cv2.bitwise_or(white_gray, white_hsv)

        # Focus lower 70% of ROI
        h = combined.shape[0]
        mask = np.zeros_like(combined)
        mask[int(h*0.3):, :] = combined[int(h*0.3):, :]
        return mask

    def _analyze(self, lines, fw, fh):
        lane_info = {}
        if lines is None or len(lines) == 0:
            self.detected_lines = 0
            self.last_theta = 0.0
            lane_info['status'] = 'NO_LINES'
            return 0.0, lane_info

        valid = []
        for l in lines:
            x1, y1, x2, y2 = l[0]
            length = math.hypot(x2-x1, y2-y1)
            if length < 10 or abs(x2 - x1) < 1:
                continue
            angle = math.atan2(y2 - y1, x2 - x1)
            if abs(math.degrees(angle)) < 45:  # lane-ish
                valid.append({'coords': (x1, y1, x2, y2), 'length': length, 'angle': angle,
                              'cx': 0.5*(x1+x2), 'cy': 0.5*(y1+y2)})

        self.detected_lines = len(valid)
        if not valid:
            lane_info['status'] = 'NO_VALID_LINES'
            return 0.0, lane_info

        centers = [v['cx'] for v in valid]
        lane_center_x = float(np.mean(centers))
        self.last_lane_center_norm = float(lane_center_x) / float(max(1.0, fw))

        frame_center_x = fw / 2.0
        offset = lane_center_x - frame_center_x
        steering_error = np.clip(offset / (fw / 2.0), -1.0, 1.0)

        # Angle weighting
        wsum, asum = 0.0, 0.0
        for v in valid:
            wsum += v['length']
            asum += v['angle'] * v['length']
        if wsum > 0:
            avg_angle = asum / wsum
            self.last_theta = avg_angle
            angle_term = (avg_angle / math.radians(45.0))
            steering_error = np.clip(0.7 * steering_error + 0.3 * angle_term, -1.0, 1.0)

        # Memory smoothing of center
        self.lane_memory.append(lane_center_x)
        if len(self.lane_memory) > self.max_memory:
            self.lane_memory.pop(0)
        if len(self.lane_memory) >= 3:
            smooth_c = np.mean(self.lane_memory[-3:])
            steering_error = np.clip(0.7 * steering_error + 0.3 * ((smooth_c - frame_center_x) / (fw / 2.0)), -1.0, 1.0)

        self.last_error = steering_error
        lane_info.update({
            'status': 'TRACKING',
            'lane_center': int(lane_center_x),
            'frame_center': int(frame_center_x),
            'offset': int(offset),
            'valid_lines': len(valid),
            'avg_angle_deg': math.degrees(self.last_theta)
        })
        return steering_error, lane_info

    def _debug_panel(self, roi, white, edges, lines, lane_info):
        h, w = roi.shape[:2]
        dbg = np.zeros((h*2, w*2, 3), dtype=np.uint8)
        dbg[0:h, 0:w] = roi
        dbg[0:h, w:w*2] = cv2.cvtColor(white, cv2.COLOR_GRAY2BGR)
        dbg[h:h*2, 0:w] = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        fin = roi.copy()
        if lines is not None:
            for l in lines:
                x1,y1,x2,y2 = l[0]
                cv2.line(fin, (x1,y1), (x2,y2), (0,255,0), 2)
        cx = w // 2
        cv2.line(fin, (cx,0), (cx,h), (255,0,0), 2)
        if 'lane_center' in lane_info:
            lc = lane_info['lane_center']
            cv2.line(fin, (lc,0), (lc,h), (0,255,255), 2)
        dbg[h:h*2, w:w*2] = fin

        info = [
            f"Lines: {self.detected_lines}",
            f"Status: {lane_info.get('status', 'Unknown')}",
            f"Error:  {self.last_error:.3f}",
            f"Angle:  {math.degrees(self.last_theta):.1f} deg"
        ]
        for i, t in enumerate(info):
            cv2.putText(dbg, t, (10, 25 + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        return dbg

    # --- Configuration ---
    def set_white_hsv_range(self, lower_hsv, upper_hsv):
        self.white_threshold_hsv_low = lower_hsv
        self.white_threshold_hsv_high = upper_hsv

    def set_canny_thresholds(self, low, high):
        self.canny_low = int(low); self.canny_high = int(high)

    def set_theta_threshold(self, threshold):
        self.theta_threshold = float(threshold)

    def tune_for_speed(self):
        self.white_threshold_gray = 160
        self.canny_low = 30
        self.canny_high = 90
        self.min_line_length = 10

    def tune_for_accuracy(self):
        self.white_threshold_gray = 200
        self.canny_low = 50
        self.canny_high = 150
        self.min_line_length = 20

    # --- Metrics for planner ---
    def get_lane_metrics(self):
        """Return lightweight metrics used by the planner."""
        conf = max(0.0, min(1.0, self.detected_lines / 4.0))
        return {
            'confidence': conf,
            'angle_deg': float(math.degrees(self.last_theta)),
            'center_x_norm': float(self.last_lane_center_norm),  # ROI-normalized
            'roi_top_frac': float(self.roi_top_frac),
            'last_valid_s': float(self._last_valid)
        }

    def get_debug_info(self):
        return {
            'last_theta': self.last_theta,
            'detected_lines': self.detected_lines,
            'theta_threshold': self.theta_threshold,
            'last_error': self.last_error,
            'white_threshold': self.white_threshold_gray
        }
