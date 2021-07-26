# pid.py
# This is an adaptation of a PID controller for Arduino found here:
# https://github.com/br3ttb/Arduino-PID-Library
# Like the original, this file is licensed under a GPLv3 License
# Adapted by Ryan Eggers, Abetter Automation Technologies Company 2017

"""
Arduino PID controller converted to Python

For more info, see the Arduino Library at:
https://github.com/br3ttb/Arduino-PID-Library
"""

import time


def limit(value, minimum, maximum):
    """Verify value is within limits, else return limit"""
    return min(max(value, minimum), maximum)


class PID(object):
    """PID Controller"""
    MANUAL = 0
    AUTOMATIC = 1
    DIRECT = 0
    REVERSE = 1

    def __init__(self, **kwargs):
        self.Input = kwargs.pop('Input', 0.0)
        self.Output = kwargs.pop('Output', 0.0)
        self.Setpoint = kwargs.pop('Setpoint', 0.0)
        self.inAuto = False
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0

        self.outMin = 0.0
        self.outMax = 0.0
        outMin = kwargs.pop('Min', self.outMin)
        outMax = kwargs.pop('Max', self.outMax)
        self.SetOutputLimits(outMin, outMax)

        self.SampleTime = kwargs.pop('SampleTime', 100)  # ms

        self.controllerDirection = kwargs.pop('Direction', PID.DIRECT)
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        kp = kwargs.pop('Kp', self.kp)
        ki = kwargs.pop('Ki', self.ki)
        kd = kwargs.pop('Kd', self.kd)
        self.SetTunings(kp, ki, kd)

        self.lastTime = time.time() - self.SampleTime / 1000.0
        self.lastInput = self.Input

    def Compute(self):
        if not self.inAuto:
            return False
        else:
            now = time.time()
            timeChange = now - self.lastTime
            if timeChange * 1000 >= self.SampleTime:
                # Compute all the working error variables
                error = self.Setpoint - self.Input
                self.PTerm = self.kp * error
                ITerm = self.ITerm + self.ki * error
                self.ITerm = limit(ITerm, self.outMin, self.outMax)
                dInput = self.Input - self.lastInput
                self.DTerm = -self.kd * dInput

                # Compute PID Output
                Output = self.PTerm + self.ITerm + self.DTerm
                self.Output = limit(Output, self.outMin, self.outMax)

                # Save variables for next time
                self.lastInput = self.Input
                self.lastTime = now
                return True
            else:
                return False

    def SetTunings(self, Kp, Ki, Kd):
        if any([k < 0 for k in (Kp, Ki, Kd)]):
            pass
        else:
            SampleTimeInSec = self.SampleTime / 1000.0
            self.kp = Kp * 1.0
            self.ki = Ki * SampleTimeInSec
            self.kd = Kd / SampleTimeInSec

            if self.controllerDirection == PID.REVERSE:
                self.kp *= -1
                self.ki *= -1
                self.kd *= -1

    def SetSampleTime(self, NewSampleTime):
        if NewSampleTime > 0:
            ratio = float(NewSampleTime) / self.SampleTime
            self.ki *= ratio
            self.kd /= ratio
            self.SampleTime = NewSampleTime

    def SetOutputLimits(self, Min, Max):
        if Min > Max:
            pass
        else:
            self.outMin = Min
            self.outMax = Max
            self.Output = limit(self.Output, self.outMin, self.outMax)
            self.ITerm = limit(self.ITerm, self.outMin, self.outMax)

    def SetMode(self, Mode):
        newAuto = (Mode == PID.AUTOMATIC)
        if newAuto and not self.inAuto:
            # switching from manual to auto
            self.lastInput = self.Input
            self.ITerm = self.Output
        self.inAuto = newAuto

    def SetControllerDirection(self, Direction):
        if Direction != self.controllerDirection:
            self.kp *= -1
            self.ki *= -1
            self.kd *= -1
        self.controllerDirection = Direction
