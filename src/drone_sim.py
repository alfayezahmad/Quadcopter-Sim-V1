import numpy as np

class PID:
    """Proportional-Integral-Derivative controller for altitude stability."""
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint: float, measured: float) -> float:
        error = setpoint - measured
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class KalmanFilter:
    """Recursive Bayesian filter for estimating state from noisy sensors."""
    def __init__(self, q: float = 0.1, r: float = 0.5):
        self.estimate = 0.0
        self.error_est = 1.0
        self.q, self.r = q, r # q: process noise, r: measurement noise

    def update(self, measurement: float) -> float:
        # Prediction & Update Cycle
        self.error_est += self.q
        k_gain = self.error_est / (self.error_est + self.r)
        self.estimate += k_gain * (measurement - self.estimate)
        self.error_est *= (1 - k_gain)
        return self.estimate