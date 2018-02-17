#!/usr/bin/env python
import time

class PIDControl():

    def __init__(self, kp, ki, kd, is_angle):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.is_angle = is_angle
        self.first = True
        self.integral_error = 0
        self.prev_error = 0

        self.error_max = 100 # 100 is completely arbitary


    def get_error_angle(self, target, current):
        diff = (target - current)%360
        if diff > 180:
            diff = diff - 360
        return diff
    

    def get_error(self, target, current):
        return target - current


    def clamp(self, val, min_val, max_val):
        if val > max_val:
            return max_val
        elif val < min_val:
            return min_val
        return val

    def get_demand(self, target, current):

        if self.first:
            self.prev_time = time.time() 
            return

        # time since last call in seconds
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        # prevent division by zero
        if dt == 0:
            return

        proportion_error = 0

        if(self.is_angle):
            proportion_error = self.get_error_angle(target, current)
        else:
            proportion_error = self.get_error(target, current)
        proportion_error = self.clamp(proportion_error, -self.error_max, self.error_max)

        self.integral_error += error * dt
        self.integral_error = clamp(integral_error, -self.error_max, self.error_max)

        derivative_error = (error - self.prev_error) / dt
        derivative_error = clamp(derivative_error, -self.error_max, self.error_max)

        demand = (self.kp * proportion_error + 
                 self.ki * self.integral_error + 
                 self.kd * derivative_error)

        return demand
