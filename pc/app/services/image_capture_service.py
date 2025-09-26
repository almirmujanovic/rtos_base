from __future__ import annotations
import time
import os
from datetime import datetime
from pathlib import Path
from typing import Optional
import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QPixmap

class ImageCaptureService(QObject):
    """Service for capturing and saving video frames for YOLOv8 training data"""
    
    # Signals
    imageSaved = Signal(str)  # Emitted when image is saved (file path)
    captureStatusChanged = Signal(bool)  # Emitted when capture starts/stops
    errorOccurred = Signal(str)  # Emitted on capture errors
    
    def __init__(self, save_directory: str = "training_images", interval_ms: int = 500):
        super().__init__()
        
        # Configuration
        self.save_directory = Path(save_directory)
        self.interval_ms = interval_ms
        self._enabled = False
        self._capturing = False
        
        # Statistics
        self._images_saved = 0
        self._last_save_time = 0
        self._start_time = 0
        
        self._original_frame: Optional[np.ndarray] = None  # Clean frame without overlay
        self._overlay_frame: Optional[np.ndarray] = None   # Frame with overlay (for display)
        self._frame_timestamp = 0
        
        # Timer for periodic saving
        self._save_timer = QTimer()
        self._save_timer.timeout.connect(self._save_current_frame)
        
        # Create save directory
        self._ensure_directory_exists()
        
    def _ensure_directory_exists(self):
        """Create save directory if it doesn't exist"""
        try:
            self.save_directory.mkdir(parents=True, exist_ok=True)
            print(f" Image save directory: {self.save_directory.absolute()}")
        except Exception as e:
            self.errorOccurred.emit(f"Failed to create directory: {e}")
    
    def set_save_directory(self, directory: str):
        """Change the save directory"""
        was_capturing = self._capturing
        if was_capturing:
            self.stop_capture()
            
        self.save_directory = Path(directory)
        self._ensure_directory_exists()
        
        if was_capturing:
            self.start_capture()
    
    def set_interval(self, interval_ms: int):
        """Change capture interval"""
        self.interval_ms = max(100, interval_ms)  # Minimum 100ms
        if self._capturing:
            self._save_timer.setInterval(self.interval_ms)
    
    def update_original_frame(self, frame: np.ndarray):
        """Update the ORIGINAL clean frame (without overlay) - USE THIS for saving"""
        if frame is not None and frame.size > 0:
            self._original_frame = frame.copy()
            self._frame_timestamp = time.time()
    
    def update_overlay_frame(self, frame: np.ndarray):
        """Update the frame WITH overlay (for display only) - DON'T save this"""
        if frame is not None and frame.size > 0:
            self._overlay_frame = frame.copy()
    
    def update_frame(self, frame: np.ndarray):
        """Legacy method - now just updates original frame"""
        self.update_original_frame(frame)
    
    def update_frame_from_pixmap(self, pixmap: QPixmap):
        """Update frame from QPixmap (for GUI integration)"""
        if pixmap and not pixmap.isNull():
            # Convert QPixmap to numpy array
            image = pixmap.toImage()
            width = image.width()
            height = image.height()
            
            # Convert to RGB format
            image = image.convertToFormat(image.Format.Format_RGB888)
            
            # Create numpy array
            ptr = image.constBits()
            arr = np.array(ptr).reshape(height, width, 3)
            
            self.update_original_frame(arr)
    
    def start_capture(self):
        """Start periodic image capture"""
        if self._capturing:
            return
            
        if not self._enabled:
            print(" Image capture is disabled")
            return
            
        print(f" Starting image capture every {self.interval_ms}ms")
        print(f" Saving CLEAN images (no overlay) to: {self.save_directory.absolute()}")
        
        self._capturing = True
        self._start_time = time.time()
        self._images_saved = 0
        
        self._save_timer.setInterval(self.interval_ms)
        self._save_timer.start()
        
        self.captureStatusChanged.emit(True)
    
    def stop_capture(self):
        """Stop periodic image capture"""
        if not self._capturing:
            return
            
        self._save_timer.stop()
        self._capturing = False
        
        duration = time.time() - self._start_time
        print(f"📸 Image capture stopped. Saved {self._images_saved} CLEAN images in {duration:.1f}s")
        
        self.captureStatusChanged.emit(False)
    
    def _save_current_frame(self):
        """Save the current ORIGINAL (clean) frame to disk"""
        #  FIXED: Save ORIGINAL frame without overlay
        if self._original_frame is None:
            return
            
        try:
            # Generate filename with timestamp
            timestamp = datetime.now()
            filename = f"frame_{timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
            filepath = self.save_directory / filename
            
            #  FIXED: Use ORIGINAL frame (BGR format for OpenCV)
            frame_to_save = self._original_frame
            
            # Convert RGB to BGR if needed (OpenCV expects BGR)
            if len(frame_to_save.shape) == 3 and frame_to_save.shape[2] == 3:
                # Check if it's RGB (from camera) or BGR (from OpenCV)
                # Most camera feeds are BGR already, but let's be safe
                if hasattr(self, '_is_rgb_format') and self._is_rgb_format:
                    bgr_frame = cv2.cvtColor(frame_to_save, cv2.COLOR_RGB2BGR)
                else:
                    bgr_frame = frame_to_save  # Already BGR
            else:
                bgr_frame = frame_to_save
            
            # Save with high quality for training data
            success = cv2.imwrite(str(filepath), bgr_frame, [
                cv2.IMWRITE_JPEG_QUALITY, 95,  # High quality for training
                cv2.IMWRITE_JPEG_OPTIMIZE, 1   # Optimize file size
            ])
            
            if success:
                self._images_saved += 1
                self._last_save_time = time.time()
                
                # Emit signal with relative path for UI
                self.imageSaved.emit(str(filepath.name))
                
                # Log progress every 10 images
                if self._images_saved % 10 == 0:
                    elapsed = time.time() - self._start_time
                    rate = self._images_saved / elapsed if elapsed > 0 else 0
                    print(f"📸 Saved {self._images_saved} CLEAN images ({rate:.1f}/s)")
                    
            else:
                self.errorOccurred.emit(f"Failed to save {filename}")
                
        except Exception as e:
            self.errorOccurred.emit(f"Save error: {e}")
    
    def save_single_frame(self, prefix: str = "manual"):
        """Save current ORIGINAL frame immediately with custom prefix"""
        if self._original_frame is None:
            self.errorOccurred.emit("No clean frame available to save")
            return
            
        try:
            timestamp = datetime.now()
            filename = f"{prefix}_{timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
            filepath = self.save_directory / filename
            
            frame_to_save = self._original_frame
            
            if len(frame_to_save.shape) == 3 and frame_to_save.shape[2] == 3:
                if hasattr(self, '_is_rgb_format') and self._is_rgb_format:
                    bgr_frame = cv2.cvtColor(frame_to_save, cv2.COLOR_RGB2BGR)
                else:
                    bgr_frame = frame_to_save
            else:
                bgr_frame = frame_to_save
            
            success = cv2.imwrite(str(filepath), bgr_frame, [
                cv2.IMWRITE_JPEG_QUALITY, 95,
                cv2.IMWRITE_JPEG_OPTIMIZE, 1
            ])
            
            if success:
                self.imageSaved.emit(str(filepath.name))
                print(f"📸 Manual save (CLEAN): {filename}")
            else:
                self.errorOccurred.emit(f"Failed to save {filename}")
                
        except Exception as e:
            self.errorOccurred.emit(f"Manual save error: {e}")
    
    # Properties
    @property
    def enabled(self) -> bool:
        return self._enabled
    
    @enabled.setter
    def enabled(self, value: bool):
        self._enabled = value
        if not value and self._capturing:
            self.stop_capture()
    
    @property
    def capturing(self) -> bool:
        return self._capturing
    
    @property
    def images_saved(self) -> int:
        return self._images_saved
    
    @property
    def save_rate(self) -> float:
        """Images per second since capture started"""
        if not self._capturing or self._start_time == 0:
            return 0.0
        elapsed = time.time() - self._start_time
        return self._images_saved / elapsed if elapsed > 0 else 0.0
    
    def get_status(self) -> dict:
        """Get detailed status information"""
        return {
            'enabled': self._enabled,
            'capturing': self._capturing,
            'images_saved': self._images_saved,
            'save_directory': str(self.save_directory.absolute()),
            'interval_ms': self.interval_ms,
            'save_rate': self.save_rate,
            'has_original_frame': self._original_frame is not None,
            'has_overlay_frame': self._overlay_frame is not None,
            'frame_age': time.time() - self._frame_timestamp if self._frame_timestamp > 0 else 0
        }
    # Add this method to the ImageCaptureService class (after save_single_frame method):

    def capture_from_joystick(self):
        """Capture single image triggered by joystick button"""
        if not self._enabled:
            print(" Image capture is disabled - enable it first")
            return
            
        if self._original_frame is None:
            print(" No frame available for joystick capture")
            return
            
        # Use joystick prefix for easy identification
        self.save_single_frame("joystick")
        print(" Joystick image captured!")
    
    def capture_with_prefix(self, prefix: str = "manual"):
        """Capture single image with custom prefix (for different trigger sources)"""
        self.save_single_frame(prefix)