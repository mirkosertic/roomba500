#!/usr/bin/env python3

import math
from tf.transformations import euler_from_quaternion

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

    def shortestAngle(self, origin, target):
        # Code taken from https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles/28037434
        diff = (target - origin + 180) % 360 - 180
        if diff < -180:
            return diff + 360
        return diff

    def currentOdometryPositionAndOrientation(self, targetframe):
        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(targetframe)
        odomQuat = odometryInTargetposeFrame.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])
        odomyawInDegrees = self.toDegrees(odomyaw)
        return odometryInTargetposeFrame.pose.position, odomyawInDegrees

