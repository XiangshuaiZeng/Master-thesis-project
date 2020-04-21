#!/usr/bin/env python

class PID:
    def __init__(self, args, ts):
        # PID parameters
        self.Kp = args["Kp"]
        self.Ki = args["Ki"]
        self.Kd = args["Kd"]
        self.ts = ts # sample time

        self.clear()  # clear up errors and output

    def clear(self):
        self.last_error = 0.0   # error from last sample
        self.output = 0.0
        self.integral = 0.0      # integral values

    def update(self, error):
        de = (error - self.last_error) / self.ts  # derivative of error
        # update temprary values
        self.integral += error
        # update output of the controller
        self.output =  self.Kp * error + self.Ki * self.integral + self.Kd * de
        self.last_error = error
        # update the output
        return self.output
