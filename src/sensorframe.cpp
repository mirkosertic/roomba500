class SensorFrame {

    public:
        int leftWheel;
        int rightWheel;
        int batteryCharge;
        int batteryCapacity;
        int bumperState;
        int lightBumperLeft;
        int lightBumperFrontLeft;
        int lightBumperCenterLeft;
        int lightBumperCenterRight;
        int lightBumperFrontRight;
        int lightBumperRight;
        int oimode;

        SensorFrame() {
            leftWheel = 0;
            rightWheel= 0;
            batteryCharge= 0;
            batteryCapacity = 0;
            bumperState = 0;
            lightBumperLeft = 0;
            lightBumperFrontLeft = 0;
            lightBumperCenterLeft = 0;
            lightBumperCenterRight = 0;
            lightBumperFrontRight = 0;
            lightBumperRight = 0;
            oimode = 0;
        }

        bool isBumperLeft() {
            return (bumperState & 2) > 0;
        }

        bool isBumperRight() {
            return (bumperState & 1) > 0;
        }

        bool isWheeldropLeft() {
            return (bumperState & 8) > 0;
        }

        bool isWheeldropRight() {
            return (bumperState & 4) > 0;
        }
};
