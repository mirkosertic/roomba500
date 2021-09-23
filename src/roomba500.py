#!/usr/bin/env python
import time

from sensorframe import SensorFrame

def twoComplementOf( value ):
    """ returns two bytes (ints) in high, low order
    whose bits form the input value when interpreted in
    two's complement
    """
    # if positive or zero, it's OK
    if value >= 0:
        eqBitVal = value
    # if it's negative, I think it is this
    else:
        eqBitVal = (1<<16) + value

    return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

def _bitOfByte( bit, byte ):
    """ returns a 0 or 1: the value of the 'bit' of 'byte' """
    if bit < 0 or bit > 7:
        print('Your bit of', bit, 'is out of range (0-7)')
        print('returning 0')
        return 0
    return ((byte >> bit) & 0x01)

def getLower5Bits(r):
    """ r is one byte as an integer """
    return [ _bitOfByte(4,r), _bitOfByte(3,r), _bitOfByte(2,r), _bitOfByte(1,r), _bitOfByte(0,r) ]

def twosComplementInt2bytes( highByte, lowByte ):
    """ returns an int which has the same value
    as the twosComplement value stored in
    the two bytes passed in

    the output range should be -32768 to 32767

    chars or ints can be input, both will be
    truncated to 8 bits
    """
    # take everything except the top bit
    topbit = _bitOfByte( 7, highByte )
    lowerbits = highByte & 127
    unsignedInt = lowerbits << 8 | (lowByte & 0xFF)
    if topbit == 1:
        # with sufficient thought, I've convinced
        # myself of this... we'll see, I suppose.
        return unsignedInt - (1 << 15)
    else:
        return unsignedInt

class Roomba500:

    def __init__(self, ser, fullRotationInSensorTicks, ticksPerCm, robotWheelDistanceInCm):
        self.ser = ser
        self.fullRotationInSensorTicks = fullRotationInSensorTicks
        self.ticksPerCm = ticksPerCm
        self.robotWheelDistanceInCm = robotWheelDistanceInCm
        self.lastBumperRight = False
        self.lastBumperLeft = False
        self.lastRightWheelDropped = False
        self.lastLeftWheelDropped = False
        self.lastKnownReferencePose = None
        self.leftWheelDistance = 0
        self.rightWheelDistance = 0

    def passiveMode(self):
        self.ser.write(chr(128))
        self.ser.write(chr(131))

    def playNote(self, note, duration):
        self.ser.write(chr(140))
        self.ser.write(chr(0))
        self.ser.write(chr(1))
        self.ser.write(chr(note))
        self.ser.write(chr(duration))
        self.ser.write(chr(141))
        self.ser.write(chr(0))

    def drive(self, speedLeft, speedRight):
        leftSpeedHigh, leftSpeedLow = twoComplementOf(speedLeft)
        rightSpeedHigh, rightSpeedLow = twoComplementOf(speedRight)
        self.ser.write(chr(145))
        self.ser.write(chr(rightSpeedHigh))
        self.ser.write(chr(rightSpeedLow))
        self.ser.write(chr(leftSpeedHigh))
        self.ser.write(chr(leftSpeedLow))

    def readSensorFrame(self):
        self.ser.write(chr(149))
        self.ser.write(chr(5))
        self.ser.write(chr(43))
        self.ser.write(chr(44))
        self.ser.write(chr(25))
        self.ser.write(chr(26))
        self.ser.write(chr(7))
        leftWheelHigh = ord(self.ser.read())
        leftWheelLow = ord(self.ser.read())
        leftWheel = leftWheelHigh << 8 | leftWheelLow
        rightWheelHigh = ord(self.ser.read())
        rightWheelLow = ord(self.ser.read())
        rightWheel = rightWheelHigh << 8 | rightWheelLow
        batteryChargeHigh = ord(self.ser.read())
        batteryChargeLow = ord(self.ser.read())
        batteryCharge = batteryChargeHigh  << 8 | batteryChargeLow
        batteryCapacityHigh = ord(self.ser.read())
        batteryCapacityLow = ord(self.ser.read())
        batteryCapacity = batteryCapacityHigh << 8 | batteryCapacityLow
        bumperState = ord(self.ser.read())
        result = SensorFrame()
        result.leftWheel = leftWheel
        result.rightWheel = rightWheel
        result.batteryCharge = batteryCharge
        result.batteryCapacity = batteryCapacity
        result.bumperState = bumperState
        return result
