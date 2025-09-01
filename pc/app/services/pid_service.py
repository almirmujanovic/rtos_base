import time

class PID:
    """
    Simple PID controller for lane following.
    """
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        
        # Internal state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        
        print(f"🎛️ PID initialized: Kp={kp}, Ki={ki}, Kd={kd}, range=[{out_min}, {out_max}]")
    
    def compute(self, error, dt=None):
        """
        Compute PID output given error and optional time delta.
        If dt is None, uses internal timing.
        """
        current_time = time.time()
        
        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time
        
        self.last_time = current_time
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        if dt > 0:
            self.integral += error * dt
            # Anti-windup: clamp integral
            integral_max = abs(self.out_max) / max(abs(self.ki), 1e-6)
            self.integral = max(-integral_max, min(integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = 0.0
        if dt > 0 and self.last_error is not None:
            derivative = (error - self.last_error) / dt
            d_term = self.kd * derivative
        
        # Combine terms
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.out_min, min(self.out_max, output))
        
        # Store for next iteration
        self.last_error = error
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        print("🔄 PID reset")
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        print(f"🎛️ PID gains updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
    
    def get_status(self):
        """Get current PID status"""
        return {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'integral': self.integral,
            'last_error': self.last_error,
            'out_range': (self.out_min, self.out_max)
        }