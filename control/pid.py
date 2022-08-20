import numpy as np
import math

class PID:

    def __init__(self):
        # Todo: testing & change to suitable value
        self.Kp = 100
        self.Ki = 30
        self.Kd = 30
        self.max_output = 1000
        self.ITerm_max = 10
        self.delta_time = 0.01  # 100Hz
        self.sensitivity = 0.02

        # init
        self.target = 0.0
        self.last_error = 0.0
        self.last_int = 0.0

    def clear(self):
        self.target = 0.0
        self.last_error = 0.0
        self.last_int = 0.0

    # Calculate P, I ,D , and output thruster command
    def update(self, sensor_value):
        delta_time = self.delta_time

        # pre-processing
        _error = self.target - sensor_value

        if abs(_error) < self.sensitivity:
            return 0
        
        if (_error < -math.pi):
            _error += 2 * math.pi
        elif (_error > math.pi):
            _error -= 2 * math.pi

        # calculate P term
        _p = self.Kp * _error

        # calculate I term
        _i = self.Ki * (_error * delta_time + self.last_int)

        if (_i < -self.ITerm_max):
            _i = -self.ITerm_max
        elif (_i > self.ITerm_max):
            _i = self.ITerm_max

        self.last_int = _i

        # calculate D term
        _d = self.Kd * (_error - self.last_error) / delta_time

        self.last_error = _error

        # calculate output
        output = _p + _i + _d

        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output

        return output

    def set_target(self, goal):
        self.target = goal