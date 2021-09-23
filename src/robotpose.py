#!/usr/bin/env python

class RobotPose:

    def __init__(self, theta, x, y, leftWheel, rightWheel, time):
        self.theta = theta
        self.x = x
        self.y = y
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.time = time
