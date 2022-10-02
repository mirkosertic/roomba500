#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <math.h>
#include <stdexcept>
#include <mutex>

#include <roomba500/RoombaSensorFrame.h>
#include <roomba500/DiffMotorSpeeds.h>

#include "roomba500.cpp"

class BaseController {
    private:
        int pollingRateInHertz;
        Roomba500* robot;
        std::mutex* mutex;
    public:
        void newCmdPWMMainBrush(const std_msgs::Int16& data) {

            mutex->lock();

            try {
                robot->mainbrushPWM = data.data;
                robot->updateMotorControl();
            } catch (const std::exception& ex) {
                ROS_ERROR("Error sending MotorControl command to Roomba. Maybe serial data problem?");
            }

            mutex->unlock();
        }

        void newCmdPWMSideBrush(const std_msgs::Int16& data) {

            mutex->lock();

            try {
                robot->sidebrushPWM = data.data;
                robot->updateMotorControl();
            } catch (const std::exception& ex) {
                ROS_ERROR("Error sending MotorControl command to Roomba. Maybe serial data problem?");
            }

            mutex->unlock();
        }

        void newCmdPWMVacuum(const std_msgs::Int16& data) {

            mutex->lock();

            ROS_DEBUG("Received vacuum pwm command %d", data.data);

            try {
                robot->vacuumPWM = data.data;
                robot->updateMotorControl();
            } catch (const std::exception& ex) {
                ROS_ERROR("Error sending MotorControl command to Roomba. Maybe serial data problem?");
            }

            mutex->unlock();
        }

        void newShutdownCommand(const std_msgs::Int16& data) {

            mutex->lock();

            ROS_DEBUG("Received shutdown command %d", data.data);

            ros::shutdown();

            mutex->unlock();
        }

        void newMotorSpeedsCommand(const ::roomba500::DiffMotorSpeeds& data) {
            mutex->lock();

            ROS_INFO("Got new DiffMotorSpeeds message: left = %d mm/s, right = %d mm/s", data.leftMillimetersPerSecond, data.rightMillimetersPerSecond);

            try {
                robot->drive((int) data.leftMillimetersPerSecond, (int) data.rightMillimetersPerSecond);
            } catch (const std::exception& ex) {
                ROS_ERROR("Error sending drive command to Roomba. Maybe serial data problem?");
            }

            mutex->unlock();
        }

        void stopRobot() {
            // We stop the robot motors here
            robot->drive(0, 0);
        }

        void initroomba() {
            // Enter safe mode
            ROS_INFO("Entering safe mode");
            robot->safeMode();
            usleep(100000); // microseconds

            // Signal welcome by playing a simple note
            robot->playNote(69, 16);

            // Reset all movement to zero, e.g. stop motors
            ROS_INFO("Stopping motors");
            robot->drive(0, 0);
            usleep(100000); // microseconds
        }

        int run(ros::NodeHandle* nPriv) {

            ros::NodeHandle n;

            std::mutex mutex;
            this->mutex = &mutex;

            std::string serialport;
            int baudrate;

            nPriv->param<std::string>("serialport", serialport, "/dev/ttyAMA0");
            nPriv->param("baudrate", baudrate, 115200);
            nPriv->param("pollingRateInHertz", pollingRateInHertz, 30);

            ROS_INFO("Connecting to Roomba 5xx on port %s with %d baud", serialport.c_str(), baudrate);
            robot = new Roomba500(serialport, baudrate);

            // Initialize Roomba
            initroomba();

            // Topic for battery charge, capacity and light bumpers etc
            ros::Publisher sensorFrameTopic = n.advertise<::roomba500::RoombaSensorFrame>("sensorframe", 1000);

            // We also consume motor control commands
            ros::Subscriber mainbrushSub = n.subscribe("cmd_mainbrush", 1000, &BaseController::newCmdPWMMainBrush, this);
            ros::Subscriber sidebrushSub = n.subscribe("cmd_sidebrush", 1000, &BaseController::newCmdPWMSideBrush, this);
            ros::Subscriber vacuumSub = n.subscribe("cmd_vacuum", 1000, &BaseController::newCmdPWMVacuum, this);
            ros::Subscriber motorSpeedsSub = n.subscribe("cmd_motorspeeds", 1000, &BaseController::newMotorSpeedsCommand, this);

            // Proper shutdown handling
            ros::Subscriber shutdownSub = n.subscribe("shutdown", 1000, &BaseController::newShutdownCommand, this);

            // Initialize sensor polling
            ROS_INFO("Polling Roomba sensors with %d hertz", pollingRateInHertz);
            ros::Rate loop_rate(pollingRateInHertz);

            ROS_INFO("Polling sensor data from robot");
            bool ignorefirstframeaftererror = false;

            // Processing the sensor polling in an endless loop until this node is shutting down
            while (ros::ok()) {

                // The main control loop. Everything is handled here
                mutex.lock();

                // Read some sensor data
                ROS_DEBUG("Getting new sensorframe");

                try {
                    SensorFrame newSensorFrame = robot->readSensorFrame();

                    if (ignorefirstframeaftererror) {
                        ROS_WARN("Ignoring first sensor frame after successful error recovery");
                        ignorefirstframeaftererror = false;
                    } else {
                        // Bumper right with debounce
                        if (newSensorFrame.isBumperRight()) {
                            if (!robot->lastBumperRight) {
                                ROS_INFO("Right bumper triggered");
                                stopRobot();

                                // Note C
                                robot->playNote(72, 16);

                                robot->lastBumperRight = true;
                            }
                        } else {
                            if (robot->lastBumperRight) {
                                robot->lastBumperRight = false;
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
                            }
                        } else {
                            if (robot->lastBumperLeft) {
                                robot->lastBumperLeft = false;
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
                            }
                        } else {
                            if (robot->lastRightWheelDropped) {
                                robot->lastRightWheelDropped = false;
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
                            }
                        } else {
                            if (robot->lastLeftWheelDropped) {
                                robot->lastLeftWheelDropped = false;
                            }
                        }

                        // Publish telemetry data such as battery charge etc.
                        ::roomba500::RoombaSensorFrame sensorFrameData = ::roomba500::RoombaSensorFrame();
                        sensorFrameData.stamp = ros::Time::now();
                        sensorFrameData.batteryCharge = newSensorFrame.batteryCharge;
                        sensorFrameData.batteryCapacity = newSensorFrame.batteryCapacity;
                        sensorFrameData.bumperLeft = newSensorFrame.isBumperLeft();
                        sensorFrameData.bumperRight = newSensorFrame.isBumperRight();
                        sensorFrameData.wheeldropLeft = newSensorFrame.isWheeldropLeft();
                        sensorFrameData.wheeldropRight = newSensorFrame.isWheeldropRight();
                        sensorFrameData.lightBumperLeft = newSensorFrame.lightBumperLeft;
                        sensorFrameData.lightBumperFrontLeft = newSensorFrame.lightBumperFrontLeft;
                        sensorFrameData.lightBumperCenterLeft = newSensorFrame.lightBumperCenterLeft;
                        sensorFrameData.lightBumperCenterRight = newSensorFrame.lightBumperCenterRight;
                        sensorFrameData.lightBumperFrontRight = newSensorFrame.lightBumperFrontRight;
                        sensorFrameData.lightBumperRight = newSensorFrame.lightBumperRight;
                        sensorFrameData.wheelEncoderLeft = newSensorFrame.leftWheel;
                        sensorFrameData.wheelEncoderRight = newSensorFrame.rightWheel;
                        sensorFrameData.lightBumperLeftStat = (newSensorFrame.lightBumperStat & 1) != 0;
                        sensorFrameData.lightBumperFrontLeftStat = (newSensorFrame.lightBumperStat & 2) != 0;
                        sensorFrameData.lightBumperCenterLeftStat = (newSensorFrame.lightBumperStat & 4) != 0;
                        sensorFrameData.lightBumperCenterRightStat = (newSensorFrame.lightBumperStat & 8) != 0;
                        sensorFrameData.lightBumperFrontRightStat = (newSensorFrame.lightBumperStat & 16) != 0;
                        sensorFrameData.lightBumperRightStat = (newSensorFrame.lightBumperStat & 32) != 0;
                        sensorFrameTopic.publish(sensorFrameData);
                    }

                } catch (const std::exception& ex) {
                    ROS_ERROR("Error getting data from Roomba. Maybe serial timeout?");

                    ignorefirstframeaftererror = true;
                }

                mutex.unlock();

                ros::spinOnce();
                loop_rate.sleep();
            }

            ROS_INFO("Entering passive mode");
            robot->passiveMode();

            ROS_INFO("Finishing");
            delete robot;

            ROS_INFO("roombainterface terminated.");
            return 0;
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roombainterface");

    ros::NodeHandle nPriv("~");

    BaseController controller;

    return controller.run(&nPriv);
}