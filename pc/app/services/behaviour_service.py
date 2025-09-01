import time

class BehaviorManager:
    def __init__(self, stop_s=2.0, turn_s=1.0, turn_bias=0.5):
        self.mode="LANE"; self.until=0.0; self.turn_dir=0.0
        self.stop_s=stop_s; self.turn_s=turn_s; self.turn_bias=turn_bias
        self.speed_cap=1.0
    def handle_sign(self, s):
        now=time.time()
        if s=="STOP":  self.mode, self.until="STOP", now+self.stop_s
        if s=="LEFT":  self.mode, self.turn_dir, self.until="TURN",-1.0, now+self.turn_s
        if s=="RIGHT": self.mode, self.turn_dir, self.until="TURN",+1.0, now+self.turn_s
        if s=="SLOW":  self.speed_cap=0.5
        if s=="FAST":  self.speed_cap=1.0
    def decide(self, steer_pid, base_throttle=0.35):
        now=time.time()
        if self.mode in ("STOP","TURN") and now>self.until:
            self.mode="LANE"; self.turn_dir=0.0
        if self.mode=="STOP":  return 0.0, 0.0
        if self.mode=="TURN":  return max(-1,min(1,steer_pid+self.turn_bias*self.turn_dir)), 0.25*self.speed_cap
        return steer_pid, base_throttle*self.speed_cap
s