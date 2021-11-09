#include <ros/ros.h>

class RobotPose {

    public:
        float theta;
        float x;
        float y;
        int leftWheel;
        int rightWheel;
        ros::Time time;

    RobotPose(float theta, float x, float y, int leftWheel, int rightWheel, ros::Time time) {
        this->theta = theta;
        this->x = x;
        this->y = y;
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->time = time;
    }

    RobotPose(RobotPose* source, ros::Time time) {
        this->theta = source->theta;
        this->x = source->x;
        this->y = source->y;
        this->leftWheel = source->leftWheel;
        this->rightWheel = source->rightWheel;
        this->time = time;
    }
};
