#!/usr/bin/env python3

class Encoder:

    def __init__(self):
        self.isReversed = False
        self.range = None
        self.lowThresh = None
        self.highThresh = None
        self.delta = None
        self.last = None
        self.setRange(-32768, 32767)
        self.initCount(0)

    def setRange(self, low, high):
        self.range = high - low + 1
        self.lowThresh = low + self.range*30//100
        self.highThresh = low + self.range*70//100

    def initCount(self, startCount):
        self.delta = 0
        self.last = startCount

    def update(self, newCount):
        if self.last > self.highThresh and newCount < self.lowThresh:
            # Wrapped around the upper limit
            increment = newCount + self.range - self.last
        elif self.last < self.lowThresh and newCount > self.highThresh:
            # Wrapped around the lower limit
            increment = newCount - self.range - self.last
        else:
            increment = newCount - self.last

        self.delta += increment
        self.last = newCount

    def setReversed(self, isReversed):
        self.isReversed = isReversed

    def getDelta(self):
        delta = self.delta
        self.delta = 0
        if self.isReversed:
            return -delta
        else:
            return delta
