from __future__ import annotations
from PySide6.QtCore import QObject, Signal, QTimer
import pygame

class JoystickService(QObject):
    joystickChanged = Signal(dict)  # {speed:int, angle:int, raw_x:float, r2:float, l2:float}

    def __init__(self):
        super().__init__()
        pygame.init(); pygame.joystick.init()
        self.deadzone = 0.1
        self.max_speed = 255
        self.min_angle = 55
        self.max_angle = 125
        self.joy = None
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0); self.joy.init()
        self.timer = QTimer(self); self.timer.setInterval(50); self.timer.timeout.connect(self._poll)
        self.timer.start()

    def _dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def _poll(self):
        if not self.joy:
            self.joystickChanged.emit({"speed":0,"angle":90,"raw_x":0.0,"r2":0.0,"l2":0.0})
            return
        pygame.event.pump()
        raw_x = self.joy.get_axis(0)
        r2 = (self.joy.get_axis(5)+1)/2 if self.joy.get_numaxes()>5 else 0
        l2 = (self.joy.get_axis(4)+1)/2 if self.joy.get_numaxes()>4 else 0
        x = self._dz(raw_x)
        fwd = r2*self.max_speed if r2>0.1 else 0
        rev = -l2*self.max_speed if l2>0.1 else 0
        speed = int(fwd if fwd>abs(rev) else rev)
        angle = int(((x+1)/2)*(self.max_angle-self.min_angle)+self.min_angle)
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.joystickChanged.emit({"speed":speed,"angle":angle,"raw_x":raw_x,"r2":r2,"l2":l2})

