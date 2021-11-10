#!/usr/bin/env python

import math

class BaseState:

    def __init__(self, pathmanager):
        self.pathmanager = pathmanager
        return


    def process(self):
        return self


    def abort(self):
        return


    def clampDegrees(self, value):
        while (value < 0):
            value += 360
        while (value >= 360):
            value -= 360

        return value


    def toDegrees(self, value):
        return self.clampDegrees(value * 180.0 / math.pi)
