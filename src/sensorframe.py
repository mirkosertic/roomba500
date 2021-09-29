#!/usr/bin/env python

class SensorFrame:

    def __init__(self):
        self.leftWheel = 0
        self.rightWheel= 0
        self.batteryCharge= 0
        self.batteryCapacity = 0
        self.bumperState = 0
        self.lightBumperLeft = 0
        self.lightBumperFrontLeft = 0
        self.lightBumperCenterLeft = 0
        self.lightBumperCenterRight = 0
        self.lightBumperFrontRight = 0
        self.lightBumperRight = 0
        self.oimode = 0

    def isBumperLeft(self):
        return self.bumperState & 2 > 0

    def isBumperRight(self):
        return self.bumperState & 1 > 0

    def isWheeldropLeft(self):
        return self.bumperState & 8 > 0

    def isWheeldropRight(self):
        return self.bumperState & 4 > 0
