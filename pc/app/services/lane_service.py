import cv2
import numpy as np
import math
import time

class LaneFollower:
    """
    WHITE LANE FOLLOWING - Optimized for following white paths/lanes.
    Focus on detecting white lines on the road for steering control.
    """
    
    def __init__(self, roi_top_frac=0.65, theta_threshold=2.5, min_line_length=15, max_line_gap=25):
        # ✅ WHITE LANE DETECTION: Optimized parameters
        self.roi_top_frac = roi_top_frac        # Use bottom 35% of frame
        self.theta_threshold = theta_threshold   # More sensitive steering
        self.min_line_length = min_line_length   # Longer minimum lines
        self.max_line_gap = max_line_gap        # Allow larger gaps
        
        # ✅ WHITE DETECTION: Dual-method approach
        self.white_threshold_gray = 180          # High threshold for bright white
        self.white_threshold_hsv_low = [0, 0, 200]   # Very bright HSV
        self.white_threshold_hsv_high = [180, 30, 255]  # Allow slight color variation
        
        # ✅ EDGE DETECTION: Optimized for white lanes
        self.canny_low = 40
        self.canny_high = 120
        
        # ✅ MORPHOLOGY: Clean up detection
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        # Processing optimization
        self.blur_kernel = (5, 5)
        
        # State tracking
        self.last_theta = 0.0
        self.detected_lines = 0
        self.last_error = 0.0
        self.last_lane_center = None
        
        # ✅ LANE CENTER TRACKING: Remember where the lane was
        self.lane_memory = []
        self.max_memory = 5
        
        # ✅ WHITE LANE FOCUS: Better parameters
        self.detection_mode = "ENHANCED_WHITE"  # Focus on white detection
        
        print(f"🛤️ WHITE LANE FOLLOWER initialized:")
        print(f"   ROI: bottom {(1-roi_top_frac)*100:.0f}% of frame")
        print(f"   White threshold: {self.white_threshold_gray}")
        print(f"   Theta sensitivity: {theta_threshold}°")
        
    def compute_error(self, frame):
        """
        ENHANCED WHITE LANE DETECTION with center tracking.
        Returns (steering_error, debug_frame)
        """
        try:
            start_time = time.time()
            h, w = frame.shape[:2]
            
            # ✅ STEP 1: Extract bottom portion (where lanes are)
            roi_start = int(self.roi_top_frac * h)
            roi = frame[roi_start:, :]
            roi_h, roi_w = roi.shape[:2]
            
            # ✅ STEP 2: Resize for processing (maintain aspect ratio)
            target_width = 320
            target_height = int((target_width / roi_w) * roi_h)
            roi_resized = cv2.resize(roi, (target_width, target_height))
            
            # ✅ STEP 3: ENHANCED WHITE DETECTION
            white_mask = self._detect_white_lanes(roi_resized)
            
            # ✅ STEP 4: Clean up the mask
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.morph_kernel)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.morph_kernel)
            
            # ✅ STEP 5: Edge detection on cleaned mask
            edges = cv2.Canny(white_mask, self.canny_low, self.canny_high)
            
            # ✅ STEP 6: Enhanced Hough line detection
            lines = cv2.HoughLinesP(
                edges,
                1,                    # rho resolution
                np.pi/180,           # theta resolution
                threshold=8,         # Higher threshold for quality
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap
            )
            
            # ✅ STEP 7: SMART LANE ANALYSIS
            steering_error, lane_info = self._analyze_lanes(lines, target_width, target_height)
            
            processing_time = (time.time() - start_time) * 1000
            
            # ✅ STEP 8: Create enhanced debug frame
            debug_frame = self._create_debug_frame(roi_resized, white_mask, edges, lines, lane_info)
            
            # ✅ Console output with lane info
            print(f"🛤️ WHITE LANE: error={steering_error:.3f}, lines={self.detected_lines}, "
                  f"center={lane_info.get('lane_center', 'None')}, "
                  f"θ={math.degrees(self.last_theta):.1f}°, {processing_time:.1f}ms")
            
            return float(steering_error), debug_frame
            
        except Exception as e:
            print(f"❌ White lane detection error: {e}")
            import traceback
            traceback.print_exc()
            return 0.0, None
    
    def _detect_white_lanes(self, roi):
        """Enhanced white detection using multiple methods"""
        # ✅ METHOD 1: Grayscale threshold (fast and effective)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # ✅ Apply Gaussian blur to reduce noise
        gray_blurred = cv2.GaussianBlur(gray, self.blur_kernel, 0)
        
        # ✅ High threshold for bright white
        _, white_gray = cv2.threshold(gray_blurred, self.white_threshold_gray, 255, cv2.THRESH_BINARY)
        
        # ✅ METHOD 2: HSV threshold (more robust to lighting)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_hsv = cv2.inRange(hsv, np.array(self.white_threshold_hsv_low), np.array(self.white_threshold_hsv_high))
        
        # ✅ COMBINE METHODS: Take best of both
        white_combined = cv2.bitwise_or(white_gray, white_hsv)
        
        # ✅ Additional enhancement: focus on bottom portion (road level)
        mask_height = white_combined.shape[0]
        focus_start = int(mask_height * 0.3)  # Bottom 70% only
        white_focused = np.zeros_like(white_combined)
        white_focused[focus_start:, :] = white_combined[focus_start:, :]
        
        return white_focused
    
    def _analyze_lanes(self, lines, frame_width, frame_height):
        """Smart analysis of detected lines to find lane center"""
        lane_info = {}
        
        if lines is None or len(lines) == 0:
            self.detected_lines = 0
            self.last_theta = 0.0
            lane_info['status'] = 'NO_LINES'
            return 0.0, lane_info
        
        # ✅ FILTER LINES: Only keep reasonable lane lines
        valid_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line properties
            length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < 10:  # Skip very short lines
                continue
                
            # Calculate angle
            if abs(x2 - x1) < 1:  # Avoid division by zero
                continue
            angle = math.atan2(y2 - y1, x2 - x1)
            angle_deg = math.degrees(angle)
            
            # ✅ LANE FILTER: Keep lines that could be lane markings
            # Accept horizontal-ish lines (-45° to +45°)
            if abs(angle_deg) < 45:
                valid_lines.append({
                    'coords': (x1, y1, x2, y2),
                    'length': length,
                    'angle': angle,
                    'center_x': (x1 + x2) / 2,
                    'center_y': (y1 + y2) / 2
                })
        
        self.detected_lines = len(valid_lines)
        
        if not valid_lines:
            lane_info['status'] = 'NO_VALID_LINES'
            return 0.0, lane_info
        
        # ✅ FIND LANE CENTER: Average of all line centers
        center_x_positions = [line['center_x'] for line in valid_lines]
        lane_center_x = np.mean(center_x_positions)
        
        # ✅ STEERING ERROR: Compare lane center to frame center
        frame_center_x = frame_width / 2
        center_offset = lane_center_x - frame_center_x
        
        # ✅ NORMALIZE ERROR: -1.0 (left) to +1.0 (right)
        max_offset = frame_width / 2
        steering_error = center_offset / max_offset
        steering_error = np.clip(steering_error, -1.0, 1.0)
        
        # ✅ ENHANCED STEERING: Add angle component
        if valid_lines:
            # Weight longer lines more heavily
            weighted_angles = []
            total_weight = 0
            for line in valid_lines:
                weight = line['length']
                weighted_angles.append(line['angle'] * weight)
                total_weight += weight
            
            if total_weight > 0:
                avg_angle = sum(weighted_angles) / total_weight
                self.last_theta = avg_angle
                
                # ✅ COMBINE: Position error + angle correction
                angle_factor = 0.3  # How much to weight the angle
                angle_error = avg_angle / math.radians(45)  # Normalize to -1..1
                steering_error = (1 - angle_factor) * steering_error + angle_factor * angle_error
                steering_error = np.clip(steering_error, -1.0, 1.0)
        
        # ✅ MEMORY: Remember lane position for stability
        self.lane_memory.append(lane_center_x)
        if len(self.lane_memory) > self.max_memory:
            self.lane_memory.pop(0)
        
        # ✅ SMOOTHING: Use recent history for stability
        if len(self.lane_memory) >= 3:
            smooth_center = np.mean(self.lane_memory[-3:])
            smooth_offset = smooth_center - frame_center_x
            smooth_error = smooth_offset / max_offset
            smooth_error = np.clip(smooth_error, -1.0, 1.0)
            
            # Blend current and smooth error
            steering_error = 0.7 * steering_error + 0.3 * smooth_error
        
        # ✅ Store info for debug
        lane_info.update({
            'status': 'TRACKING',
            'lane_center': int(lane_center_x),
            'frame_center': int(frame_center_x),
            'offset': int(center_offset),
            'valid_lines': len(valid_lines),
            'avg_angle_deg': math.degrees(self.last_theta) if hasattr(self, 'last_theta') else 0
        })
        
        self.last_error = steering_error
        return steering_error, lane_info
    
    def _create_debug_frame(self, roi, white_mask, edges, lines, lane_info):
        """Create comprehensive debug visualization"""
        # ✅ 4-PANEL DEBUG VIEW
        h, w = roi.shape[:2]
        debug_frame = np.zeros((h*2, w*2, 3), dtype=np.uint8)
        
        # Panel 1: Original ROI
        debug_frame[0:h, 0:w] = roi
        
        # Panel 2: White detection mask
        white_color = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        debug_frame[0:h, w:w*2] = white_color
        
        # Panel 3: Edge detection
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        debug_frame[h:h*2, 0:w] = edges_color
        
        # Panel 4: Final analysis with lines
        final_panel = roi.copy()
        
        # Draw detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(final_panel, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw lane center and frame center
        frame_center_x = w // 2
        cv2.line(final_panel, (frame_center_x, 0), (frame_center_x, h), (255, 0, 0), 2)  # Blue: frame center
        
        if 'lane_center' in lane_info:
            lane_center_x = lane_info['lane_center']
            cv2.line(final_panel, (lane_center_x, 0), (lane_center_x, h), (0, 255, 255), 2)  # Yellow: lane center
            
            # Draw offset arrow
            cv2.arrowedLine(final_panel, (frame_center_x, h//2), (lane_center_x, h//2), (0, 0, 255), 3)
        
        debug_frame[h:h*2, w:w*2] = final_panel
        
        # ✅ Add text information
        info_text = [
            f"Lines: {self.detected_lines}",
            f"Status: {lane_info.get('status', 'Unknown')}",
            f"Center: {lane_info.get('lane_center', 'N/A')}",
            f"Offset: {lane_info.get('offset', 'N/A')}",
            f"Error: {self.last_error:.3f}"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(debug_frame, text, (10, 25 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return debug_frame
    
    # ===== CONFIGURATION METHODS =====
    
    def set_white_hsv_range(self, lower_hsv, upper_hsv):
        """Set HSV range for white detection"""
        self.white_threshold_hsv_low = lower_hsv
        self.white_threshold_hsv_high = upper_hsv
        print(f"🛤️ HSV white range: {lower_hsv} to {upper_hsv}")
    
    def set_canny_thresholds(self, low, high):
        """Set Canny edge detection thresholds"""
        self.canny_low = low
        self.canny_high = high
        print(f"🛤️ Canny thresholds: {low}, {high}")
    
    def set_theta_threshold(self, threshold):
        """Set steering sensitivity"""
        self.theta_threshold = threshold
        print(f"🛤️ Steering sensitivity: {threshold}°")
    
    def tune_for_speed(self):
        """Optimize for maximum processing speed"""
        self.white_threshold_gray = 160        # Lower for more detection
        self.canny_low = 30
        self.canny_high = 90
        self.min_line_length = 10
        print("🚀 Tuned for SPEED")
    
    def tune_for_accuracy(self):
        """Optimize for accurate white lane detection"""
        self.white_threshold_gray = 200        # Higher for precise white
        self.canny_low = 50
        self.canny_high = 150
        self.min_line_length = 20
        print("🎯 Tuned for WHITE LANE ACCURACY")
    
    def get_debug_info(self):
        """Get debug information"""
        return {
            'last_theta': self.last_theta,
            'detected_lines': self.detected_lines,
            'theta_threshold': self.theta_threshold,
            'last_error': self.last_error,
            'white_threshold': self.white_threshold_gray,
            'detection_mode': self.detection_mode,
        }