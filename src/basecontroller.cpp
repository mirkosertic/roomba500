#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sstream>
#include <string>
#include <unistd.h>

#include "roomba500.cpp"

class BaseController {
    private:
        std::string serialport;
        int baudrate;
        int fullRotationInSensorTicks;
        float ticksPerCm;
        float robotWheelDistanceInCm;
        int pollingRateInHertz;
        Roomba500* robot;
    public:
        void publishOdometry(RobotPose* pose) {
        }

        void estimateAndPublishPose() {
        }

        void publishFinalPose() {
        }

        void stopRobot() {
            publishFinalPose();
            robot->drive(0, 0);
        }

        void publishInt16(int value, ros::Publisher* publisher) {
            std_msgs::Int16 msg;
            msg.data = value;
            publisher->publish(msg);
        }

        int overflowSafeWheelRotation(int rotationDelta) {
            if (rotationDelta < -16384) {
                // Rotation forward with overflow
                return rotationDelta + 65536;
            }
            if (rotationDelta > 16384) {
                // Rotation backward with overflow
                return rotationDelta - 65536;
            }

            return rotationDelta;
        }

        int run() {
            ros::NodeHandle n;

            n.param<std::string>("serialport", serialport, "'/dev/serial0'");
            n.param("baudrate", baudrate, 115200);
            n.param("fullRotationInSensorTicks", fullRotationInSensorTicks, 1608);
            n.param("ticksPerCm", ticksPerCm, 22.5798f);
            n.param("robotWheelDistanceInCm", robotWheelDistanceInCm, 25.0f);
            n.param("pollingRateInHertz", pollingRateInHertz, 30);

            ROS_INFO("Connecting to Roomba 5xx on port %s with %d baud", serialport.c_str(), baudrate);
            robot = new Roomba500(0, fullRotationInSensorTicks, ticksPerCm, robotWheelDistanceInCm);

            // Enter safe mode
            ROS_INFO("Entering safe mode");
            robot->safeMode();
            usleep(100000); // microseconds

            // Signal welcome by playing a simple note
            robot->playNote(69, 16);

            // Reset all movement to zero, e.g. stop motors
            ROS_INFO("Stopping motors");
            robot->drive(0, 0);

            // Initialize the last known reference pose with a position
            // and the current values of the wheel encoders
            ROS_INFO("Computing initial reference pose");
            SensorFrame lastSensorFrame = robot->readSensorFrame();
            robot->lastKnownReferencePose = new RobotPose(0, 0, 0, robot->leftWheelDistance, robot->rightWheelDistance, ros::Time());

            // Topics for battery charge, capacity and light bumpers
            ros::Publisher batteryChargeTopic = n.advertise<std_msgs::Int16>("batteryCharge", 1000);
            ros::Publisher batteryCapacityTopic = n.advertise<std_msgs::Int16>("batteryCapacity", 1000);
            ros::Publisher bumperLeftTopic = n.advertise<std_msgs::Int16>("bumperLeft", 1000);
            ros::Publisher bumperRightTopic = n.advertise<std_msgs::Int16>("bumperRight", 1000);
            ros::Publisher wheeldropLeftTopic = n.advertise<std_msgs::Int16>("wheeldropLeft", 1000);
            ros::Publisher wheeldropRightTopic = n.advertise<std_msgs::Int16>("wheeldropRight", 1000);

            ros::Publisher lightBumperLeftTopic = n.advertise<std_msgs::Int16>("lightBumperLeft", 1000);
            ros::Publisher lightBumperFrontLeftTopic = n.advertise<std_msgs::Int16>("lightBumperFrontLeft", 1000);
            ros::Publisher lightBumperCenterLeftTopic = n.advertise<std_msgs::Int16>("lightBumperCenterLeft", 1000);
            ros::Publisher lightBumperCenterRightTopic = n.advertise<std_msgs::Int16>("lightBumperCenterRight", 1000);
            ros::Publisher lightBumperFrontRightTopic = n.advertise<std_msgs::Int16>("lightBumperFrontRight", 1000);
            ros::Publisher lightBumperRightTopic = n.advertise<std_msgs::Int16>("lightBumperRight", 1000);

            ros::Publisher oimodeTopic = n.advertise<std_msgs::Int16>("oimode", 1000);

            // Here goes the odometry data
            // self.odomTopic = rospy.Publisher('odom', Odometry, queue_size = 10)

            // Initialize sensor polling
            ROS_INFO("Polling Roomba sensors with %d hertz", pollingRateInHertz);
            ros::Rate loop_rate(pollingRateInHertz);

            // We start at the last known reference pose
            publishOdometry(robot->lastKnownReferencePose);

            // Processing the sensor polling in an endless loop until this node goes to die
            while (ros::ok()) {

                //self.syncLock.acquire()

                // Read some sensor data
                ROS_INFO("Getting new sensorframe");

                SensorFrame newSensorFrame = robot->readSensorFrame();

                // Bumper right with debounce
                if (newSensorFrame.isBumperRight()) {
                    if (!robot->lastBumperRight) {
                        ROS_INFO("Right bumper triggered");
                        stopRobot();

                        // Note C
                        robot->playNote(72, 16);

                        robot->lastBumperRight = true;
                        publishInt16(1, &bumperRightTopic);
                    }
                } else {
                    if (robot->lastBumperRight) {
                        robot->lastBumperRight = false;
                        publishInt16(0, &bumperRightTopic);
                    }
                }

                // Bumper left with debounce
                if (newSensorFrame.isBumperLeft()) {
                    if (!robot->lastBumperLeft) {
                        ROS_INFO("Left bumper triggered");
                        stopRobot();

                        // Note D
                        robot->playNote(74, 16);

                        robot->lastBumperLeft = true;
                        publishInt16(1, &bumperLeftTopic);
                    }
                } else {
                    if (robot->lastBumperLeft) {
                        robot->lastBumperLeft = false;
                        publishInt16(0, &bumperLeftTopic);
                    }
                }

                // Right wheel drop with debounce
                if (newSensorFrame.isWheeldropRight()) {
                    if (!robot->lastRightWheelDropped) {
                        ROS_INFO("Right wheel dropped");
                        stopRobot();

                        // Note E
                        robot->playNote(76, 16);

                        robot->lastRightWheelDropped = true;
                        publishInt16(1, &wheeldropRightTopic);
                    }
                } else {
                    if (robot->lastRightWheelDropped) {
                        robot->lastRightWheelDropped = false;
                        publishInt16(0, &wheeldropRightTopic);
                    }
                }

                // Left wheel drop with debounce
                if (newSensorFrame.isWheeldropLeft()) {
                    if (!robot->lastLeftWheelDropped) {
                        ROS_INFO("Left wheel dropped");
                        stopRobot();

                        // Note F
                        robot->playNote(77, 16);

                        robot->lastLeftWheelDropped = true;
                        publishInt16(1, &wheeldropLeftTopic);
                    }
                } else {
                    if (robot->lastLeftWheelDropped) {
                        robot->lastLeftWheelDropped = false;
                        publishInt16(0, &wheeldropLeftTopic);
                    }
                }

                // Calculate the relative movement to last sensor data
                int deltaLeft = overflowSafeWheelRotation(newSensorFrame.leftWheel - lastSensorFrame.leftWheel);
                int deltaRight = overflowSafeWheelRotation(newSensorFrame.rightWheel - lastSensorFrame.rightWheel);

                ROS_INFO("Last wheel left = %d, last wheel right = %d, current wheel left = %d, current wheel right = %d",
                              lastSensorFrame.leftWheel, lastSensorFrame.rightWheel, newSensorFrame.leftWheel, newSensorFrame.rightWheel);

                ROS_INFO("Delta rotation left is %d, right is %d",
                              deltaLeft,
                              deltaRight);

                // Estimate a pose and publish information
                robot->leftWheelDistance += deltaLeft;
                robot->rightWheelDistance += deltaRight;

                ROS_DEBUG("Estimating new position");
                estimateAndPublishPose();

                // Remember last sensor data for the next iteration
                lastSensorFrame = newSensorFrame;

                // Publish telemetry data such as battery charge etc.
                publishInt16(lastSensorFrame.batteryCharge, &batteryChargeTopic);
                publishInt16(lastSensorFrame.batteryCapacity, &batteryCapacityTopic);

                publishInt16(lastSensorFrame.lightBumperLeft, &lightBumperLeftTopic);
                publishInt16(lastSensorFrame.lightBumperFrontLeft, &lightBumperFrontLeftTopic);
                publishInt16(lastSensorFrame.lightBumperCenterLeft, &lightBumperCenterLeftTopic);
                publishInt16(lastSensorFrame.lightBumperCenterRight, &lightBumperCenterRightTopic);
                publishInt16(lastSensorFrame.lightBumperFrontRight, &lightBumperFrontRightTopic);
                publishInt16(lastSensorFrame.lightBumperRight, &lightBumperRightTopic);

                publishInt16(lastSensorFrame.oimode, &oimodeTopic);

                //self.syncLock.release()

                ros::spinOnce();
                loop_rate.sleep();
            }

            delete robot;

            return 0;
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roomba500basecontroller");

    BaseController controller;
    return controller.run();
}