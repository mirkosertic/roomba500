#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include "robotpose.cpp"
#include "sensorframe.cpp"

class Roomba500 {

    public:
        int fd;
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
        char mainbrushPWM;
        char sidebrushPWM;
        char vacuumPWM;

    Roomba500(std::string device, int baudrate, int fullRotationInSensorTicks, float ticksPerCm, float robotWheelDistanceInCm) {
        // Open and initialize serial interface
        // Code taken from https://www.cmrr.umn.edu/~strupp/serial.html
        //this->fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        this->fd = open(device.c_str(), O_RDWR | O_NOCTTY);
        if (this->fd == -1) {
            throw std::invalid_argument("cannot open serial interface");
        } else {
            // Read blocking behavior
            fcntl(this->fd, F_SETFL, 0);
        }

        // Set serial port baud rate
        struct termios options;
        tcgetattr(fd, &options);

        if (baudrate == 115200) {
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
        } else {
            throw std::invalid_argument("unsupported baud rate");
        }

        options.c_cflag     |= (CLOCAL | CREAD);
        options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag     &= ~OPOST;
        options.c_cc[VMIN]  = 1; // Blocking read, as min 1 bytes in required in input buffer
        options.c_cc[VTIME] = 10;

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Configure interface
        tcsetattr(fd, TCSANOW, &options);

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

        unsigned char data[] = {128, 131};

        int written = write(this->fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending safeMode command");
        }
        tcdrain(this->fd);
        //self.ser.write(chr(128))
        //self.ser.write(chr(131))
    }

    void passiveMode() {
        unsigned char data[] = {128};
        int written = write(this->fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending passiveMode command");
        }
        tcdrain(this->fd);
        //self.ser.write(chr(128))
    }

    void updateMotorControl() {
        unsigned char data[] = {144, (unsigned char) (this->mainbrushPWM & 0xff), (unsigned char) (this->sidebrushPWM & 0xff), (unsigned char) (this->vacuumPWM & 0xff)};
        int written = write(this->fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error updating motor control");
        }
        tcdrain(this->fd);
        //self.ser.write(chr(144))
        //self.ser.write(chr(self.mainbrushPWM & 0xff))
        //self.ser.write(chr(self.sidebrushPWM & 0xff))
        //self.ser.write(chr(self.vacuumPWM & 0xff))
    }

    void playNote(unsigned char note, unsigned char duration) {
        unsigned char data[] = {140, 0, 1, note, duration, 141, 0};
        int written = write(this->fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error playing note");
        }
        tcdrain(this->fd);
        //self.ser.write(chr(140))
        //self.ser.write(chr(0))
        //self.ser.write(chr(1))
        //self.ser.write(chr(note))
        //self.ser.write(chr(duration))
        //self.ser.write(chr(141))
        //self.ser.write(chr(0))
    }

    void drive(int speedLeft, int speedRight) {

        ROS_INFO("Deive left %d right %d", speedLeft, speedRight);

        unsigned int speedLeftUnsigned;
        if (speedLeft < 0) {
            speedLeftUnsigned = (1<<16) + speedLeft;
        } else {
            speedLeftUnsigned = speedLeft;
        }

        unsigned int speedRightUnsigned;
        if (speedRight < 0) {
            speedRightUnsigned = (1<<16) + speedRight;
        } else {
            speedRightUnsigned = speedRight;
        }

        unsigned char data[] = {145, (unsigned char)((speedRightUnsigned >> 8) & 0xff), (unsigned char)(speedRightUnsigned & 0xff),
                                     (unsigned char)((speedLeftUnsigned >> 8) & 0xff), (unsigned char)(speedLeftUnsigned & 0xff)};
        int written = write(this->fd, data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending drive command");
        }
        tcdrain(this->fd);

        //leftSpeedHigh, leftSpeedLow = twoComplementOf(speedLeft)
        //rightSpeedHigh, rightSpeedLow = twoComplementOf(speedRight)
        //self.ser.write(chr(145))
        //self.ser.write(chr(rightSpeedHigh))
        //self.ser.write(chr(rightSpeedLow))
        //self.ser.write(chr(leftSpeedHigh))
        //self.ser.write(chr(leftSpeedLow))
    }

    unsigned char readData() {
        unsigned char result;
        int readBytes = read(this->fd, &result, 1);
        return result;
    }

    SensorFrame readSensorFrame() {

        tcflush(this->fd, TCIFLUSH);

        unsigned char data[] = {149, 12, 43, 44, 25, 26, 7, 46, 47, 48, 49, 50, 51, 35};
        int written = write(this->fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("sending read sensors command");
        }
        tcdrain(this->fd);

        int bytesAvailable = 0;
        ioctl(fd, FIONREAD, &bytesAvailable);

        SensorFrame sensorFrame = SensorFrame();
        sensorFrame.leftWheel = readData() << 8 | readData(); // packet 43
        sensorFrame.rightWheel = readData() << 8 | readData(); // packet 44
        sensorFrame.batteryCharge = readData() << 8 | readData(); // packet 25
        sensorFrame.batteryCapacity = readData() << 8 | readData(); // packet 26
        sensorFrame.bumperState = readData(); // packet 7
        sensorFrame.lightBumperLeft = readData() << 8 | readData(); // packet 46
        sensorFrame.lightBumperFrontLeft = readData() << 8 | readData(); // packet 47
        sensorFrame.lightBumperCenterLeft = readData() << 8 | readData(); // packet 48
        sensorFrame.lightBumperCenterRight = readData() << 8 | readData(); // packet 49
        sensorFrame.lightBumperFrontRight = readData() << 8 | readData(); // packet 50
        sensorFrame.lightBumperRight = readData() << 8 | readData(); // packet 51
        sensorFrame.oimode = readData(); // packet 35

        return sensorFrame;

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
    }

    ~Roomba500() {
        close(this->fd);
    }
};