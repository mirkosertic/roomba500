#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <math.h>
#include <stdexcept>
#include <mutex>

#include "roomba500.cpp"

class BaseController {
    private:
        int fullRotationInSensorTicks;
        float ticksPerCm;
        float robotWheelDistanceInCm;
        int pollingRateInHertz;
        Roomba500* robot;
        ros::Publisher* odomTopic;
        tf::TransformBroadcaster* transform_broadcaster;
        std::mutex* mutex;
    public:

        void publishOdometry(RobotPose* pose) {
            double deltaTimeInSeconds = (pose->time - robot->lastKnownReferencePose->time).toSec();

            //if (deltaTimeInSeconds == 0.0) {
            //    ROS_INFO("Skipping odometry as delta time is zero");
            //    return;
            //}

            ROS_DEBUG("Publishing odometry, delta time is %f seconds", deltaTimeInSeconds);

            // The robot can only move forward ( x - direction in base_link coordinate frame )
            float distanceX = pose->x - robot->lastKnownReferencePose->x;
            float distanceY = pose->y - robot->lastKnownReferencePose->y;
            float linearDistanceInMeters = sqrt(distanceX * distanceX + distanceY * distanceY);

            //float vxInMetersPerSecond = linearDistanceInMeters / deltaTimeInSeconds;
            //float vyInMetersPerSecond = .0f;
            //float vthInRadiansPerSecond = -((pose->theta - robot->lastKnownReferencePose->theta) * M_PI / 180) / deltaTimeInSeconds;

            //if (vyInMetersPerSecond != 0.0f || vthInRadiansPerSecond != 0.0f) {
            //    ROS_INFO("Current velocity vx = %f, vtheta = %f", vxInMetersPerSecond, vthInRadiansPerSecond);
            //}

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(radians(pose->theta));

            // Publish transform
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = pose->time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = pose->x;
            odom_trans.transform.translation.y = pose->y;
            odom_trans.transform.translation.z = .0f;
            odom_trans.transform.rotation = odom_quat;

            transform_broadcaster->sendTransform(odom_trans);

            // Publish odometry
            nav_msgs::Odometry odom;
            odom.header.stamp = pose->time;
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = pose -> x;
            odom.pose.pose.position.y = pose -> y;
            odom.pose.pose.position.z = .0f;
            odom.pose.pose.orientation = odom_quat;
            odom.child_frame_id = "base_link";
            //odom.twist.twist.linear.x = vxInMetersPerSecond;
            //odom.twist.twist.linear.y = vyInMetersPerSecond;
            //odom.twist.twist.angular.z = vthInRadiansPerSecond;

            odomTopic->publish(odom);
        }

        float radians(float degrees) {
            return degrees * M_PI / 180.0f;
        }

        RobotPose* estimateAndPublishPose() {
            ROS_DEBUG("Estimating new pose with leftWheelDistance = %d and rightWheelDistance = %d",
                          robot->leftWheelDistance, robot->rightWheelDistance);
            ROS_DEBUG("Last known reference pose is at x = %f m and y = %f m, theta = %f degrees",
                          robot->lastKnownReferencePose->x, robot->lastKnownReferencePose->y, robot->lastKnownReferencePose->theta);

            // First case
            // Rotation on the spot
            // one wheel rotation is positive, the other is negative
            // If they sum up roughly to zero, the robot rotated on the spot
            int sumOfWheelEncoders = robot->leftWheelDistance + robot->rightWheelDistance;
            if (sumOfWheelEncoders < 20) {
                // Roomba rotated on the spot
                float rotationInDegrees = ((float) robot->leftWheelDistance) / (robot->fullRotationInSensorTicks / 360.0);

                ROS_DEBUG("Assuming rotation on the spot, as sum of wheel encoders is %d. Rotation is %f degrees",
                              sumOfWheelEncoders, rotationInDegrees);

                RobotPose* poseestimation = new RobotPose(robot->lastKnownReferencePose->theta - rotationInDegrees,
                                           robot->lastKnownReferencePose->x,
                                           robot->lastKnownReferencePose->y,
                                           robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

                publishOdometry(poseestimation);
                return poseestimation;
            } else {
                // Second case
                // Robot moved in a straight line
                // Both wheels rotate in the same direction with roughly the same distance
                int diffOfWheelEncoders = robot->leftWheelDistance - robot->rightWheelDistance;
                if (abs(diffOfWheelEncoders) < 20) {
                    // Robot moved in a straight line
                    float averageMovementInTicks = (robot->leftWheelDistance + robot->rightWheelDistance) / 2.0f;
                    float averageMovementInCm = averageMovementInTicks / robot->ticksPerCm;

                    float deltaXInMeters = cos(radians(robot->lastKnownReferencePose->theta)) * averageMovementInCm / 100.0f;
                    float deltaYInMeters = sin(radians(robot->lastKnownReferencePose->theta)) * averageMovementInCm / 100.0f;

                    ROS_DEBUG("Assuming movement in straight line, as difference of wheel encoders is %d. distance is %f cm, dx = %f meters, dy = %f meters"
                                  , diffOfWheelEncoders, averageMovementInCm, deltaXInMeters, deltaYInMeters);

                    RobotPose* poseestimation = new RobotPose(robot->lastKnownReferencePose->theta,
                                               robot->lastKnownReferencePose->x + deltaXInMeters,
                                               robot->lastKnownReferencePose->y + deltaYInMeters,
                                               robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

                    publishOdometry(poseestimation);
                    return poseestimation;

                } else {
                    // Third case
                    // Rotation to the right. The left wheel moves further than the right wheel
                    if (robot->leftWheelDistance > robot->rightWheelDistance) {
                        // Robot rotated to the right
                        float L = robot->robotWheelDistanceInCm / 100.0f;

                        float leftWheelDistanceInM = robot->leftWheelDistance / robot->ticksPerCm / 100.0f;
                        float rightWheelDistanceInM = robot->rightWheelDistance / robot->ticksPerCm / 100.0f;

                        float radiusInMeter = L / ((leftWheelDistanceInM / rightWheelDistanceInM) - 1.0f);
                        float deltaTheta = rightWheelDistanceInM * 360 / (2.0f + M_PI * radiusInMeter);

                        ROS_DEBUG("Assuming rotation to the right with radius of %f m and angle of %f degrees",
                                      radiusInMeter, deltaTheta);

                        float rotationRadiusToCenterInMeter = radiusInMeter + L / 2;

                        float rotationPointX = robot->lastKnownReferencePose->x + sin(radians(robot->lastKnownReferencePose->theta)) * rotationRadiusToCenterInMeter;
                        float rotationPointY = robot->lastKnownReferencePose->y - cos(radians(robot->lastKnownReferencePose->theta)) * rotationRadiusToCenterInMeter;

                        ROS_DEBUG("Assuming rotation point is at x = %f, y = %f", rotationPointX, rotationPointY);

                        float newPositionX = rotationPointX + sin(radians(deltaTheta)) * rotationRadiusToCenterInMeter;
                        float newPositionY = rotationPointY + cos(radians(deltaTheta)) * rotationRadiusToCenterInMeter;

                        float newTheta = robot->lastKnownReferencePose->theta - deltaTheta;

                        ROS_DEBUG("Estimated position is x = %f, y = %f, theta = %f", newPositionX, newPositionY, newTheta);

                        RobotPose* poseestimation = new RobotPose(newTheta,
                                                   newPositionX,
                                                   newPositionY,
                                                   robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

                        publishOdometry(poseestimation);
                        return poseestimation;
                    }

                    // Fourth case
                    // Rotation to the left. The right wheel moves further than the left wheel
                    if (robot->rightWheelDistance > robot->leftWheelDistance) {
                        // Robot rotated to the left
                        float L = robot->robotWheelDistanceInCm / 100.0f;

                        float leftWheelDistanceInM = robot->leftWheelDistance / robot->ticksPerCm / 100.0f;
                        float rightWheelDistanceInM = robot->rightWheelDistance / robot->ticksPerCm / 100.0f;

                        float radiusInMeter = L / ((rightWheelDistanceInM / leftWheelDistanceInM) - 1.0);
                        float deltaTheta = leftWheelDistanceInM * 360 / (2.0f + M_PI * radiusInMeter);

                        ROS_DEBUG("Assuming rotation to the left with radius of %f m and angle of %f degrees",
                                      radiusInMeter, deltaTheta);

                        float rotationRadiusToCenterInMeter = radiusInMeter + L / 2;

                        float rotationPointX = robot->lastKnownReferencePose->x + sin(radians(robot->lastKnownReferencePose->theta)) * rotationRadiusToCenterInMeter;
                        float rotationPointY = robot->lastKnownReferencePose->y + cos(radians(robot->lastKnownReferencePose->theta)) * rotationRadiusToCenterInMeter;

                        ROS_DEBUG("Assuming rotation point is at x = %f, y = %f", rotationPointX, rotationPointY);

                        float newPositionX = rotationPointX + sin(radians(deltaTheta)) * rotationRadiusToCenterInMeter;
                        float newPositionY = rotationPointY - cos(radians(deltaTheta)) * rotationRadiusToCenterInMeter;

                        float newTheta = robot->lastKnownReferencePose->theta + deltaTheta;

                        ROS_DEBUG("Estimated position is x = %f, y = %f, theta = %f", newPositionX, newPositionY, newTheta);

                        RobotPose* poseestimation = new RobotPose(newTheta,
                                                   newPositionX,
                                                   newPositionY,
                                                   robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

                        publishOdometry(poseestimation);
                        return poseestimation;
                    }
                }
            }
            // Don't know how to handle
            throw std::invalid_argument("Don't know how to handle");
        }

        void publishFinalPose() {

            ROS_INFO("Last reference pose x = %f, y = %f, theta = %f", robot->lastKnownReferencePose->x, robot->lastKnownReferencePose->y, robot->lastKnownReferencePose->theta);

            RobotPose* temp  = estimateAndPublishPose();

            ROS_INFO("Final delta rotation left is %d, right is %d",
                      overflowSafeWheelRotation(temp->leftWheel - robot->lastKnownReferencePose->leftWheel),
                      overflowSafeWheelRotation(temp->rightWheel - robot->lastKnownReferencePose->rightWheel));

            ROS_INFO("Final pose x = %f, y = %f, theta = %f", temp->x, temp->y, temp->theta);

            double deltaTimeInSeconds = (temp->time - robot->lastKnownReferencePose->time).toSec();

            ROS_INFO("Delta time since last reference pose is %f seconds", deltaTimeInSeconds);

            delete robot->lastKnownReferencePose;
            robot->lastKnownReferencePose = temp;

            robot->leftWheelDistance = 0;
            robot->rightWheelDistance = 0;
        }

        void stopRobot() {
            robot->drive(0, 0);
            publishFinalPose();
        }

        void newCmdVelMessage(const geometry_msgs::Twist& data) {

            ROS_INFO("Received cmd_vel message");
            this->mutex->lock();

            publishFinalPose();

            float speedInMeterPerSecond = data.linear.x; // Meter per second
            float rotationInRadiansPerSecond = data.angular.z; // Radians per second

            ROS_INFO("Linear speed x      : %f m/sec", speedInMeterPerSecond);
            ROS_INFO("Angular speed z     : %f rad/sec", rotationInRadiansPerSecond);

            if (speedInMeterPerSecond == .0f && rotationInRadiansPerSecond == .0f) {
                robot->drive(0, 0);
            } else {
                if (rotationInRadiansPerSecond == .0f) {
                    float mmPerSecond = speedInMeterPerSecond * 100 * 10;

                    if (mmPerSecond > 500) {
                        mmPerSecond = 500;
                    }
                    if (mmPerSecond < - 500) {
                        mmPerSecond = -500;
                    }

                    robot->drive((int)mmPerSecond, (int)mmPerSecond);
                } else {
                    float degreePerSecond = rotationInRadiansPerSecond * 180.0f / M_PI;

                    float mmPerSecond = robot->fullRotationInSensorTicks / 360.0f * degreePerSecond;
                    if (mmPerSecond > 500) {
                        mmPerSecond = 500;
                    }
                    if (mmPerSecond < - 500) {
                        mmPerSecond = -500;
                    }

                    robot->drive((int) -mmPerSecond, (int) mmPerSecond);
                }
            }

            this->mutex->unlock();
        }

        void newCmdPWMMainBrush(const std_msgs::Int16& data) {

            this->mutex->lock();

            robot->mainbrushPWM = data.data;
            robot->updateMotorControl();

            this->mutex->unlock();
        }

        void newCmdPWMSideBrush(const std_msgs::Int16& data) {

            this->mutex->lock();

            robot->sidebrushPWM = data.data;
            robot->updateMotorControl();

            this->mutex->unlock();
        }

        void newCmdPWMVacuum(const std_msgs::Int16& data) {

            this->mutex->lock();

            ROS_DEBUG("Received vacuum pwm command %d", data.data);

            robot->vacuumPWM = data.data;
            robot->updateMotorControl();

            this->mutex->unlock();
        }

        void publishInt16(int value, ros::Publisher* publisher) {
            std_msgs::Int16 msg;
            msg.data = value;
            publisher->publish(msg);
        }

        int overflowSafeWheelRotation(int rotationDelta) {
            if (rotationDelta < -16384) {
                ROS_INFO("Forward rotation with overflow : %d", rotationDelta);
                // Rotation forward with overflow
                return rotationDelta + 65536;
            }
            if (rotationDelta > 16384) {
                ROS_INFO("Backward rotation with overflow : %d", rotationDelta);
                // Rotation backward with overflow
                return rotationDelta - 65536;
            }

            return rotationDelta;
        }

        int run(ros::NodeHandle* nPriv) {
            ros::NodeHandle n;

            std::mutex mutex;
            this->mutex = &mutex;

            std::string serialport;
            int baudrate;

            nPriv->param<std::string>("serialport", serialport, "/dev/serial0");
            nPriv->param("baudrate", baudrate, 115200);
            nPriv->param("fullRotationInSensorTicks", fullRotationInSensorTicks, 1608);
            nPriv->param("ticksPerCm", ticksPerCm, 22.5798f);
            nPriv->param("robotWheelDistanceInCm", robotWheelDistanceInCm, 25.0f);
            nPriv->param("pollingRateInHertz", pollingRateInHertz, 30);

            ROS_INFO("Full rotation in ticks     : %d", fullRotationInSensorTicks);
            ROS_INFO("Ticks per cm               : %f", ticksPerCm);
            ROS_INFO("Robot wheel distance in cm : %f", robotWheelDistanceInCm);

            ROS_INFO("Connecting to Roomba 5xx on port %s with %d baud", serialport.c_str(), baudrate);
            robot = new Roomba500(serialport, baudrate, fullRotationInSensorTicks, ticksPerCm, robotWheelDistanceInCm);

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
            robot->lastKnownReferencePose = new RobotPose(0, 0, 0, robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

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
            ros::Publisher odom = n.advertise<nav_msgs::Odometry>("odom", 50);
            odomTopic = &odom;

            // Used for the tf broadcasting
            tf::TransformBroadcaster broadcaster;
            transform_broadcaster = &broadcaster;

            // We consume steering commands from here
            ros::Subscriber cmdVelSub = n.subscribe("cmd_vel", 1000, &BaseController::newCmdVelMessage, this);

            // We also consume motor control commands
            ros::Subscriber mainbrushSub = n.subscribe("cmd_mainbrush", 1000, &BaseController::newCmdPWMMainBrush, this);
            ros::Subscriber sidebrushSub = n.subscribe("cmd_sidebrush", 1000, &BaseController::newCmdPWMSideBrush, this);
            ros::Subscriber vacuumSub = n.subscribe("cmd_vacuum", 1000, &BaseController::newCmdPWMVacuum, this);

            // Initialize sensor polling
            ROS_INFO("Polling Roomba sensors with %d hertz", pollingRateInHertz);
            ros::Rate loop_rate(pollingRateInHertz);

            // We start at the last known reference pose
            publishOdometry(robot->lastKnownReferencePose);

            // Processing the sensor polling in an endless loop until this node goes to die
            while (ros::ok()) {

                this->mutex->lock();

                // Read some sensor data
                ROS_DEBUG("Getting new sensorframe");

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

                ROS_DEBUG("Last wheel left = %d, last wheel right = %d, current wheel left = %d, current wheel right = %d",
                              lastSensorFrame.leftWheel, lastSensorFrame.rightWheel, newSensorFrame.leftWheel, newSensorFrame.rightWheel);

                ROS_DEBUG("Delta rotation left is %d, right is %d",
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

                this->mutex->unlock();

                ros::spinOnce();
                loop_rate.sleep();
            }

            delete robot;

            return 0;
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roomba500basecontroller");

    ros::NodeHandle nPriv("~");

    BaseController controller;

    return controller.run(&nPriv);
}