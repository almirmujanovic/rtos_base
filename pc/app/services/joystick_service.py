from __future__ import annotations
from PySide6.QtCore import QObject, Signal, QTimer
import pygame

class JoystickService(QObject):
    # {speed:int, angle:int, raw_x:float, r2:float, l2:float, force:bool}
    joystickChanged = Signal(dict)
    
    # ✅ NEW: Signal for single image capture
    captureImageRequested = Signal()  # Emitted on button press for image capture

    def __init__(self):
        super().__init__()
        pygame.init(); pygame.joystick.init()
        self.deadzone = 0.1
        self.max_speed = 255
        self.min_angle = 25
        self.max_angle = 155

        # Button mapping (PS4 default: L1=4, R1=5). Change here if your index differs.
        self.force_button_index = 4  # L1 - Force/crawl mode
        self.capture_button_index = 1  # R1 - Image capture (single shot)
        
        self.last_capture_button_state = False

        self.joy = None
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0); self.joy.init()
            print(f" Joystick connected: {self.joy.get_name()}")
            print(f"   L1 (Button {self.force_button_index}): Force/crawl mode")
            print(f"   R1 (Button {self.capture_button_index}): Capture image")

        self.timer = QTimer(self); self.timer.setInterval(50); self.timer.timeout.connect(self._poll)
        self.timer.start()

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def _poll(self):
        if not self.joy:
            self.joystickChanged.emit({
                "speed": 0, "angle": 87, "raw_x": 0.0, "r2": 0.0, "l2": 0.0, "force": False
            })
            return

        pygame.event.pump()

        raw_x = self.joy.get_axis(0)
        r2 = (self.joy.get_axis(5)+1)/2 if self.joy.get_numaxes()>5 else 0.0
        l2 = (self.joy.get_axis(4)+1)/2 if self.joy.get_numaxes()>4 else 0.0

        # PS4 L1 as force/crawl (boolean)
        force = False
        try:
            force = bool(self.joy.get_button(self.force_button_index))
        except Exception:
            force = False

        capture_button_pressed = False
        try:
            current_capture_state = bool(self.joy.get_button(self.capture_button_index))
            
            # Detect button press (transition from not pressed to pressed)
            if current_capture_state and not self.last_capture_button_state:
                capture_button_pressed = True
                print(" Joystick capture button pressed!")
                self.captureImageRequested.emit()
            
            self.last_capture_button_state = current_capture_state
            
        except Exception as e:
            print(f" Capture button error: {e}")
            capture_button_pressed = False

        x = self._dz(raw_x)
        fwd = r2*self.max_speed if r2>0.1 else 0.0
        rev = -l2*self.max_speed if l2>0.1 else 0.0
        speed = int(fwd if fwd>abs(rev) else rev)

        angle = int(((x+1)/2)*(self.max_angle-self.min_angle)+self.min_angle)
        angle = max(self.min_angle, min(self.max_angle, angle))

        self.joystickChanged.emit({
            "speed": speed, 
            "angle": angle, 
            "raw_x": raw_x, 
            "r2": r2, 
            "l2": l2, 
            "force": force,
            "capture_pressed": capture_button_pressed 
        })
    
    def set_capture_button(self, button_index: int):
        """Change which button is used for image capture"""
        self.capture_button_index = button_index
        self.last_capture_button_state = False  # Reset state
        print(f" Image capture button set to: {button_index}")
    
    def set_force_button(self, button_index: int):
        """Change which button is used for force/crawl mode"""
        self.force_button_index = button_index
        print(f" Force/crawl button set to: {button_index}")