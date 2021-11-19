#!/usr/bin/env python

import math

class BaseState:

    def __init__(self, pathmanager, successLambda, errorLambda):
        self.pathmanager = pathmanager
        self.successLambda = successLambda
        self.errorLambda = errorLambda
        return


    def success(self):
        return self.successLambda(self)


    def error(self):
        return self.errorLambda(self)


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
        return self.clampDegrees(math.degrees(value))


    def distanceToDegrees(self, dx, dy):
        return self.toDegrees(math.atan2(dy, dx))

