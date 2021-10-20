//#include <wiringPi.h>

#include "robotpose.cpp"
#include "sensorframe.cpp"

class Roomba500 {

    public:
        int ser;
        int fullRotationInSensorTicks;
        float ticksPerCm;
        float robotWheelDistanceInCm;
        bool lastBumperRight;
        bool lastBumperLeft;
        bool lastRightWheelDropped;
        bool lastLeftWheelDropped;
        RobotPose* lastKnownReferencePose;
        int leftWheelDistance;
        int rightWheelDistance;
        int mainbrushPWM;
        int sidebrushPWM;
        int vacuumPWM;

    Roomba500(int ser, int fullRotationInSensorTicks, float ticksPerCm, float robotWheelDistanceInCm) {
        this->ser = ser;
        this->fullRotationInSensorTicks = fullRotationInSensorTicks;
        this->ticksPerCm = ticksPerCm;
        this->robotWheelDistanceInCm = robotWheelDistanceInCm;
        this->lastBumperRight = false;
        this->lastBumperLeft = false;
        this->lastRightWheelDropped = false;
        this->lastLeftWheelDropped = false;
        this->lastKnownReferencePose = (RobotPose*) 0;
        this->leftWheelDistance = 0;
        this->rightWheelDistance = 0;
        this->mainbrushPWM = 0;
        this->sidebrushPWM = 0;
        this->vacuumPWM = 0;
    }

    void safeMode() {
        //self.ser.write(chr(128))
        //self.ser.write(chr(131))
    }

    void passiveMode() {
        //self.ser.write(chr(128))
    }

    void updateMotorControl() {
        //self.ser.write(chr(144))
        //self.ser.write(chr(self.mainbrushPWM & 0xff))
        //self.ser.write(chr(self.sidebrushPWM & 0xff))
        //self.ser.write(chr(self.vacuumPWM & 0xff))
    }

    void playNote(int note, int duration) {
        //self.ser.write(chr(140))
        //self.ser.write(chr(0))
        //self.ser.write(chr(1))
        //self.ser.write(chr(note))
        //self.ser.write(chr(duration))
        //self.ser.write(chr(141))
        //self.ser.write(chr(0))
    }

    void drive(int speedLeft, int speedRight) {
        //leftSpeedHigh, leftSpeedLow = twoComplementOf(speedLeft)
        //rightSpeedHigh, rightSpeedLow = twoComplementOf(speedRight)
        //self.ser.write(chr(145))
        //self.ser.write(chr(rightSpeedHigh))
        //self.ser.write(chr(rightSpeedLow))
        //self.ser.write(chr(leftSpeedHigh))
        //self.ser.write(chr(leftSpeedLow))
    }

    SensorFrame readSensorFrame() {
        //self.ser.write(chr(149))
        //self.ser.write(chr(12))
        //self.ser.write(chr(43))
        //self.ser.write(chr(44))
        //self.ser.write(chr(25))
        //self.ser.write(chr(26))
        //self.ser.write(chr(7))

        //self.ser.write(chr(46))
        //self.ser.write(chr(47))
        //self.ser.write(chr(48))
        //self.ser.write(chr(49))
        //self.ser.write(chr(50))
        //self.ser.write(chr(51))

        //self.ser.write(chr(35))

        //leftWheel = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //rightWheel = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //batteryCharge = ord(self.ser.read())  << 8 | ord(self.ser.read())
        //batteryCapacity = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //bumperState = ord(self.ser.read())

        //lightBumpLeft = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //lightBumpFrontLeft = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //lightBumpCenterLeft = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //lightBumpCenterRight = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //lightBumpFrontRight = ord(self.ser.read()) << 8 | ord(self.ser.read())
        //lightBumpRight = ord(self.ser.read()) << 8 | ord(self.ser.read())

        //oimode = ord(self.ser.read())

        // result = SensorFrame()
        //result.leftWheel = leftWheel
        //result.rightWheel = rightWheel
        //result.batteryCharge = batteryCharge
        //result.batteryCapacity = batteryCapacity
        //result.bumperState = bumperState
        //result.lightBumperLeft = lightBumpLeft
        //result.lightBumperFrontLeft = lightBumpFrontLeft
        //result.lightBumperCenterLeft = lightBumpCenterLeft
        //result.lightBumperCenterRight = lightBumpCenterRight
        //result.lightBumperFrontRight = lightBumpFrontRight
        //result.lightBumperRight = lightBumpRight
        //result.oimode = oimode

        return SensorFrame();
    }
};