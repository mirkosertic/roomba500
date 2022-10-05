#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include "sensorframe.cpp"

class Roomba500 {

    private:
        int fd;

    public:
        bool lastBumperRight;
        bool lastBumperLeft;
        bool lastRightWheelDropped;
        bool lastLeftWheelDropped;
        char mainbrushPWM;
        char sidebrushPWM;
        char vacuumPWM;

    Roomba500(std::string device, int baudrate) {
        // Open and initialize serial interface
        // Code taken from https://www.cmrr.umn.edu/~strupp/serial.html
        //this->fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        this->fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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

        this->lastBumperRight = false;
        this->lastBumperLeft = false;
        this->lastRightWheelDropped = false;
        this->lastLeftWheelDropped = false;
        this->mainbrushPWM = 0;
        this->sidebrushPWM = 0;
        this->vacuumPWM = 0;
    }

    void safeMode() {

        unsigned char data[] = {128, 131};

        int written = write(fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending safeMode command");
        }
        tcdrain(fd);
    }

    void passiveMode() {
        unsigned char data[] = {128};
        int written = write(fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending passiveMode command");
        }
        tcdrain(fd);
    }

    void updateMotorControl() {
        unsigned char data[] = {131, 144, (unsigned char) (mainbrushPWM & 0xff), (unsigned char) (sidebrushPWM & 0xff), (unsigned char) (vacuumPWM & 0xff)};
        int written = write(fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error updating motor control");
        }
        tcdrain(fd);
    }

    void playNote(unsigned char note, unsigned char duration) {
        unsigned char data[] = {131, 140, 0, 1, note, duration, 141, 0};
        int written = write(fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error playing note");
        }
        tcdrain(fd);
    }

    void drive(int speedLeft, int speedRight) {

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

        unsigned char data[] = {131, 145, (unsigned char)((speedRightUnsigned >> 8) & 0xff), (unsigned char)(speedRightUnsigned & 0xff),
                                     (unsigned char)((speedLeftUnsigned >> 8) & 0xff), (unsigned char)(speedLeftUnsigned & 0xff)};
        int written = write(fd, data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("error sending drive command");
        }
        tcdrain(fd);
    }

    unsigned char readData() {
        unsigned char result;
        int readBytes = read(fd, &result, 1);
        return result;
    }

    void set_port_read_min_size(int min_size) {
        struct termios settings;

        if (tcgetattr(fd, &settings)) {
           /* handle error */
            throw std::invalid_argument("cannot read settings for serial interface");
        }

        /* set the minimimum no. of chracters to read in each
         * read call.
         */
        settings.c_cc[VMIN]  = min_size;
        /* set “read timeout between characters” as 100 ms.
         * read returns either when the specified number of chars
         * are received or the timout occurs */
        settings.c_cc[VTIME] = 1; /* 1 * .1s */

        if (tcsetattr (fd, TCSANOW, &settings)) {
            throw std::invalid_argument("cannot write settings for serial interface");
        }
    }

    SensorFrame readSensorFrame() {

        char rawdata[23] __attribute((aligned(32)));

        tcflush(fd, TCIFLUSH);

        unsigned char data[] = {131, 149, 13, 43, 44, 25, 26, 7, 46, 47, 48, 49, 50, 51, 35, 45};
        int written = write(fd, &data, sizeof(data));
        if (written != sizeof(data)) {
            throw std::invalid_argument("sending read sensors command");
        }
        tcdrain(fd);

        // Wait for response or timeout
        set_port_read_min_size(23);

        int bytesAvailable = 0;
        ioctl(fd, FIONREAD, &bytesAvailable);

        SensorFrame sensorFrame = SensorFrame();

        int recvbytes = 0;
        int maxfd = fd + 1;
        int index = 0;
        /* set the timeout as 1 sec for each read */
        struct timeval timeout = {0, 30 * 1000}; // 30ms timeout
        fd_set readSet;

        FD_ZERO(&readSet);
        FD_SET(fd, &readSet);
        if (select(maxfd, &readSet, NULL, NULL, &timeout) > 0) {
            if (FD_ISSET(fd, &readSet)) {
                recvbytes = read(fd, &rawdata, sizeof(rawdata));
                if(recvbytes) {
                    sensorFrame.leftWheel = rawdata[0] << 8 | rawdata[1]; // packet 43
                    sensorFrame.rightWheel = rawdata[2] << 8 | rawdata[3]; // packet 44
                    sensorFrame.batteryCharge = rawdata[4] << 8 | rawdata[5]; // packet 25
                    sensorFrame.batteryCapacity = rawdata[6] << 8 | rawdata[7]; // packet 26
                    sensorFrame.bumperState = rawdata[8]; // packet 7
                    sensorFrame.lightBumperLeft = rawdata[9] << 8 | rawdata[10]; // packet 46
                    sensorFrame.lightBumperFrontLeft = rawdata[11] << 8 | rawdata[12]; // packet 47
                    sensorFrame.lightBumperCenterLeft = rawdata[13] << 8 | rawdata[14]; // packet 48
                    sensorFrame.lightBumperCenterRight = rawdata[15] << 8 | rawdata[16]; // packet 49
                    sensorFrame.lightBumperFrontRight = rawdata[17] << 8 | rawdata[18]; // packet 50
                    sensorFrame.lightBumperRight = rawdata[19] << 8 | rawdata[20]; // packet 51
                    sensorFrame.oimode = rawdata[21]; // packet 35
                    sensorFrame.lightBumperStat = rawdata[22]; // packet 45
                } else {
                    throw std::invalid_argument("failed to read data");
                }

            }
        } else {
            /* select() – returns 0 on timeout and -1 on error condtion */
            throw std::invalid_argument("timeout or error while waiting for data");
        }

        return sensorFrame;
    }

    ~Roomba500() {
        close(fd);
    }
};