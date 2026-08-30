import time

# No I for now
class PDController:
    def __init__(self, kp: float = 0.0, kd: float = 0.0, max_derivative=500):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0.0
        self.previous_time = None
        self.max_derivative = max_derivative

    def compute(self, setpoint: float, measurement: float) -> float:
        current_time = time.time()
        
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time
            
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Derivative term (rate of change of error)
        if dt > 0.0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        
        d_term = self.kd * self.clamp(derivative, -self.max_derivative, self.max_derivative)
        
        # Save state for next step
        self.previous_error = error
        self.previous_time = current_time
        
        return p_term + d_term

    def clamp(self, n, minn, maxn):
        return max(minn, min(n, maxn))

    def reset(self):
        self.previous_error = 0.0
        self.previous_time = None
