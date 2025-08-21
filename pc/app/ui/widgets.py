from __future__ import annotations
from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QLinearGradient
from PySide6.QtWidgets import QWidget
import os

class CarSensorView(QWidget):
    """
    Top-down car view with 4 sensors:
      - vl53 (front center)
      - hcsr04_1 (front-left)
      - hcsr04_2 (front-right)
      - hcsr04_3 (back center)

    NEW:
      • Unified lower section design
      • Proportional car sizing based on original image
      • Clean, no-boxy sensor data display
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.setAutoFillBackground(True)
        self._d = {'vl53': 0, 'hcsr04_1': 0, 'hcsr04_2': 0, 'hcsr04_3': 0}
        self._has_data = False
        self._car_pix: QPixmap | None = None
        
        # Try to load car.png from multiple possible locations
        self._load_car_image()

    def _load_car_image(self):
        """Try to load car.png from various possible locations"""
        possible_paths = [
            # Same directory as this file
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.png"),
            # Current working directory
            os.path.join(os.getcwd(), "car.png"),
            # UI directory relative paths
            "car.png",
            "ui/car.png",
            "app/ui/car.png",
            "pc/app/ui/car.png"
        ]
        
        for car_path in possible_paths:
            if os.path.exists(car_path):
                pix = QPixmap(car_path)
                if not pix.isNull():
                    self._car_pix = pix
                    print(f"✓ Successfully loaded car image from: {car_path}")
                    print(f"  Image size: {pix.width()}x{pix.height()}")
                    return

    def set_car_image(self, path: str):
        """Manually set car image path"""
        if os.path.exists(path):
            pix = QPixmap(path)
            if not pix.isNull():
                self._car_pix = pix
                self.update()
                return True
        return False

    def set_data_available(self, ok: bool):
        self._has_data = bool(ok)
        self.update()

    def update_sensors(self, d:dict):
        self._d.update(d); self.update()

    def _draw_sensor_beam(self, painter, start_x, start_y, end_x, end_y, distance, beam_width=40):
        """Draw a realistic sensor beam with gradient and proper width"""
        if not self._has_data or distance <= 0:
            # No data - draw neutral beam
            painter.setPen(QPen(QColor(90, 90, 100, 100), 3))
            painter.drawLine(start_x, start_y, end_x, end_y)
            return

        # Calculate beam color based on distance
        if distance < 20:
            color = QColor(255, 70, 70, 180)      # Red - danger
        elif distance < 60:
            color = QColor(255, 170, 60, 180)     # Orange - warning  
        else:
            color = QColor(60, 220, 150, 180)     # Green - safe

        # Create gradient for realistic beam effect
        gradient = QLinearGradient(start_x, start_y, end_x, end_y)
        gradient.setColorAt(0.0, QColor(color.red(), color.green(), color.blue(), 200))
        gradient.setColorAt(0.7, QColor(color.red(), color.green(), color.blue(), 120))
        gradient.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 60))

        # Draw the main beam as a thick line with gradient
        pen = QPen(QBrush(gradient), beam_width, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(start_x, start_y, end_x, end_y)

        # Add a thinner bright center line for more realism
        center_pen = QPen(QColor(255, 255, 255, 150), max(2, beam_width // 8), Qt.SolidLine, Qt.RoundCap)
        painter.setPen(center_pen)
        painter.drawLine(start_x, start_y, end_x, end_y)

    def _draw_angular_sensor_beam(self, painter, start_x, start_y, angle_deg, distance, max_length, beam_width=35):
        """Draw an angled sensor beam (for corner sensors)"""
        import math
        angle_rad = math.radians(angle_deg)
        
        # Calculate beam length based on distance
        if distance <= 0:
            beam_length = max_length * 0.3  # Show minimal beam when no data
        else:
            val = max(0, min(255, int(distance)))
            beam_length = int((val / 255.0) * max_length)
        
        end_x = start_x + int(beam_length * math.cos(angle_rad))
        end_y = start_y + int(beam_length * math.sin(angle_rad))
        
        self._draw_sensor_beam(painter, start_x, start_y, end_x, end_y, distance, beam_width)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        
        # Keep the consistent background color throughout
        p.fillRect(self.rect(), QColor(14, 14, 16))

        # Layout: left text panel (40%) | right car panel (60%)
        left_w = int(w * 0.42)
        right_x = left_w + 8
        right_w = w - right_x - 8

        # ---- Left sensor data panel (clean, no background box) ----
        pad = 20
        header_pen = QPen(QColor(140, 200, 255))
        text_pen = QPen(QColor(220, 220, 230))
        muted_pen = QPen(QColor(130, 130, 140))

        # Clean header without underline box
        p.setPen(header_pen)
        p.drawText(pad, 35, "PROXIMITY SENSORS")
        
        # Subtle separator line only
        p.setPen(QPen(QColor(60, 60, 70)))
        p.drawLine(pad, 45, left_w - pad, 45)

        # Sensor data rows - clean text layout
        p.setPen(text_pen if self._has_data else muted_pen)
        rows = [
            ("Front Center", self._d.get('vl53', 0)),
            ("Front Left", self._d.get('hcsr04_1', 0)),
            ("Front Right", self._d.get('hcsr04_2', 0)),
            ("Rear Center", self._d.get('hcsr04_3', 0)),
        ]
        
        y = 75
        for name, val in rows:
            if self._has_data:
                # Color-coded values based on distance
                if val < 20:
                    value_color = QColor(255, 100, 100)  # Red for close
                elif val < 60:
                    value_color = QColor(255, 180, 80)   # Orange for medium
                else:
                    value_color = QColor(100, 220, 150)  # Green for safe
                
                p.setPen(text_pen)
                p.drawText(pad, y, f"{name}")
                p.setPen(value_color)
                p.drawText(pad + 120, y, f"{val:3d} cm")
            else:
                p.setPen(muted_pen)
                p.drawText(pad, y, f"{name}")
                p.drawText(pad + 120, y, "---")
            y += 28

        if not self._has_data:
            p.setPen(QPen(QColor(255, 120, 120)))
            p.drawText(pad, y + 15, "⚠ NO SENSOR DATA")

        # ---- Right car visualization section ----
        # Calculate car dimensions while maintaining aspect ratio
        available_height = int(h * 0.6)  # Reduced height as requested
        max_car_width = int(right_w * 0.7)
        
        if self._car_pix is not None and not self._car_pix.isNull():
            # Use original image proportions
            original_ratio = self._car_pix.width() / self._car_pix.height()
            car_h = available_height
            car_w = int(car_h * original_ratio)
            
            # Ensure it fits within available width
            if car_w > max_car_width:
                car_w = max_car_width
                car_h = int(car_w / original_ratio)
        else:
            # Fallback proportions for drawn car
            car_h = available_height
            car_w = int(car_h * 0.6)  # Typical car ratio

        # Center the car in the right panel
        cx = right_x + right_w // 2
        cy = int(h * 0.55)
        car_x = cx - car_w // 2
        car_y = cy - car_h // 2

        # Calculate sensor beam ranges
        max_front = int(h * 0.35)
        max_back = int(h * 0.25)

        # Helper for distance scaling
        def scale_distance(val, max_px):
            val = max(0, min(255, int(val)))
            return int((val / 255.0) * max_px)

        # Draw sensor beams BEFORE the car (so car appears on top)
        
        # FRONT center beam (vl53) - wider and more prominent
        front_dist = self._d.get('vl53', 0)
        front_length = scale_distance(front_dist, max_front)
        front_start_y = car_y
        front_end_y = car_y - front_length
        self._draw_sensor_beam(p, cx, front_start_y, cx, front_end_y, front_dist, beam_width=50)

        # FRONT-LEFT beam (hcsr04_1) - angled at -45 degrees
        fl_dist = self._d.get('hcsr04_1', 0)
        fl_start_x = cx - car_w // 2 + 15
        fl_start_y = car_y + int(car_h * 0.15)
        self._draw_angular_sensor_beam(p, fl_start_x, fl_start_y, -135, fl_dist, max_front, beam_width=40)

        # FRONT-RIGHT beam (hcsr04_2) - angled at +45 degrees  
        fr_dist = self._d.get('hcsr04_2', 0)
        fr_start_x = cx + car_w // 2 - 15
        fr_start_y = car_y + int(car_h * 0.15)
        self._draw_angular_sensor_beam(p, fr_start_x, fr_start_y, -45, fr_dist, max_front, beam_width=40)

        # BACK center beam (hcsr04_3)
        back_dist = self._d.get('hcsr04_3', 0)
        back_length = scale_distance(back_dist, max_back)
        back_start_y = car_y + car_h
        back_end_y = car_y + car_h + back_length
        self._draw_sensor_beam(p, cx, back_start_y, cx, back_end_y, back_dist, beam_width=45)

        # Draw the car (on top of sensor beams)
        if self._car_pix is not None and not self._car_pix.isNull():
            # Scale and draw the car image maintaining proportions
            target = QRectF(car_x, car_y, car_w, car_h)
            p.drawPixmap(target, self._car_pix, self._car_pix.rect())
        else:
            # Fallback: draw a simple car shape if no image
            p.setBrush(QBrush(QColor(58, 58, 62)))
            p.setPen(QPen(QColor(180, 180, 190), 2))
            p.drawRoundedRect(car_x, car_y, car_w, car_h, 12, 12)
            
            # Front arrow to show direction
            p.setPen(QPen(QColor(210, 210, 215), 3))
            p.drawLine(cx, car_y, cx, car_y - 18)
            p.drawLine(cx, car_y - 18, cx - 8, car_y - 6)
            p.drawLine(cx, car_y - 18, cx + 8, car_y - 6)

        # Add distance labels on the beams (like real infotainment systems)
        if self._has_data:
            label_pen = QPen(QColor(255, 255, 255, 220))
            p.setPen(label_pen)
            
            # Front center label
            if front_dist > 0:
                label_y = front_end_y - 15
                p.drawText(cx - 20, label_y, f"{front_dist}cm")
            
            # Back center label  
            if back_dist > 0:
                label_y = back_end_y + 20
                p.drawText(cx - 20, label_y, f"{back_dist}cm")