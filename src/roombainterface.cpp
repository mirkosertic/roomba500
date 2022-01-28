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

#include <roomba500/RoombaSensorFrame.h>
#include <roomba500/DiffMotorSpeeds.h>

#include "roomba500.cpp"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
        float normalizeAngleInDegrees(float angle) {
            while (angle >= 360) {
                angle-= 360;
            }
            while (angle < 0) {
                angle+= 360;
            }
            return angle;
        }

        void publishOdometry(RobotPose* pose) {
            double deltaTimeInSeconds = (pose->time - robot->lastKnownReferencePose->time).toSec();

            ROS_DEBUG("Publishing odometry, delta time is %f seconds", deltaTimeInSeconds);

            // The robot can only move forward ( x - direction in base_link coordinate frame )
            float distanceX = pose->x - robot->lastKnownReferencePose->x;
            float distanceY = pose->y - robot->lastKnownReferencePose->y;
            float linearDistanceInMeters = sqrt(distanceX * distanceX + distanceY * distanceY);

            float vxInMetersPerSecond = 0.0f;
            float vyInMetersPerSecond = .0f;
            float vthInRadiansPerSecond = 0.0f;

            if (deltaTimeInSeconds > 0) {
                vxInMetersPerSecond = linearDistanceInMeters / deltaTimeInSeconds;
                vthInRadiansPerSecond = -((pose->theta - robot->lastKnownReferencePose->theta) * M_PI / 180) / deltaTimeInSeconds;
                vthInRadiansPerSecond = 0.0f;
            }

            if (vyInMetersPerSecond != 0.0f || vthInRadiansPerSecond != 0.0f) {
                ROS_DEBUG("Current velocity vx = %f, vtheta = %f", vxInMetersPerSecond, vthInRadiansPerSecond);
            }

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
            odom.twist.twist.linear.x = vxInMetersPerSecond;
            odom.twist.twist.linear.y = vyInMetersPerSecond;
            odom.twist.twist.angular.z = vthInRadiansPerSecond;

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
            int signOfLeftWheelDelta = sgn(robot->leftWheelDistance);
            int signOfRightWheelDelta = sgn(robot->rightWheelDistance);
            int sumOfWheelEncoders = robot->leftWheelDistance + robot->rightWheelDistance;
            if (sumOfWheelEncoders < 20 && (signOfLeftWheelDelta != signOfRightWheelDelta) && signOfRightWheelDelta != 0 && signOfRightWheelDelta != 0) {
                // Roomba rotated on the spot
                float rotationInDegrees = ((float) robot->leftWheelDistance) / (robot->fullRotationInSensorTicks / 360.0);

                ROS_DEBUG("Assuming rotation on the spot, as sum of wheel encoders is %d. Rotation is %f degrees",
                              sumOfWheelEncoders, rotationInDegrees);

                RobotPose* poseestimation = new RobotPose(normalizeAngleInDegrees(robot->lastKnownReferencePose->theta - rotationInDegrees),
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

                        float newTheta = normalizeAngleInDegrees(robot->lastKnownReferencePose->theta - deltaTheta);

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

                        float newTheta = normalizeAngleInDegrees(robot->lastKnownReferencePose->theta + deltaTheta);

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
                      robot->overflowSafeWheelRotation(temp->leftWheel - robot->lastKnownReferencePose->leftWheel),
                      robot->overflowSafeWheelRotation(temp->rightWheel - robot->lastKnownReferencePose->rightWheel));

            ROS_INFO("Final pose x = %f, y = %f, theta = %f", temp->x, temp->y, temp->theta);

            double deltaTimeInSeconds = (temp->time - robot->lastKnownReferencePose->time).toSec();

            ROS_INFO("Delta time since last reference pose is %f seconds", deltaTimeInSeconds);

            delete robot->lastKnownReferencePose;
            robot->lastKnownReferencePose = temp;

            robot->leftWheelDistance = 0;
            robot->rightWheelDistance = 0;
        }

        void stopRobot() {
            // We stop the robot motors here
            robot->drive(0, 0);
            // Pose estimation is done in the main control loop
            // As soon as there is no movement in the wheel encoders detected
            // a final odom pose update is published
        }

        void newCmdVelMessage(const geometry_msgs::Twist& data) {

            ROS_INFO("Received cmd_vel message");
            mutex->lock();

            float speedInMeterPerSecond = data.linear.x; // Meter per second
            float rotationInRadiansPerSecond = data.angular.z; // Radians per second

            ROS_INFO("Linear speed x      : %f m/sec", speedInMeterPerSecond);
            ROS_INFO("Angular speed z     : %f rad/sec", rotationInRadiansPerSecond);

            // Robot movement is stopped
            // Final pose and odom update is handled in the main control loop
            robot->drive(0, 0);
            robot->resetQueue();

            if (speedInMeterPerSecond == .0f && rotationInRadiansPerSecond == .0f) {
                // Do nothing here
            } else {
                float mmPerSecond = speedInMeterPerSecond * 100 * 10;
                if (mmPerSecond > 350) {
                    mmPerSecond = 350;
                }
                if (mmPerSecond < - 350) {
                    mmPerSecond = -350;
                }

                float robotRadiusAroundCenterInMillimeters = (robot->robotWheelDistanceInCm * 10) / 2;
                float speedLeftWheel = mmPerSecond - (rotationInRadiansPerSecond * robotRadiusAroundCenterInMillimeters);
                float speedRightWheel = mmPerSecond + (rotationInRadiansPerSecond * robotRadiusAroundCenterInMillimeters);

                ROS_INFO("Robot radius        : %f mm", robotRadiusAroundCenterInMillimeters);
                ROS_INFO("Speed left wheel    : %f mm/s", speedLeftWheel);
                ROS_INFO("Speed right wheel   : %f mm/s", speedRightWheel);

                robot->enqueueCommand((int) speedLeftWheel, (int) speedRightWheel);
            }

            mutex->unlock();
        }

        void newCmdPWMMainBrush(const std_msgs::Int16& data) {

            mutex->lock();

            robot->mainbrushPWM = data.data;
            robot->updateMotorControl();

            mutex->unlock();
        }

        void newCmdPWMSideBrush(const std_msgs::Int16& data) {

            mutex->lock();

            robot->sidebrushPWM = data.data;
            robot->updateMotorControl();

            mutex->unlock();
        }

        void newCmdPWMVacuum(const std_msgs::Int16& data) {

            mutex->lock();

            ROS_DEBUG("Received vacuum pwm command %d", data.data);

            robot->vacuumPWM = data.data;
            robot->updateMotorControl();

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

            robot->drive((int) data.leftMillimetersPerSecond, (int) data.rightMillimetersPerSecond);

            mutex->unlock();
        }

        int run(ros::NodeHandle* nPriv) {

            ros::NodeHandle n;

            std::mutex mutex;
            this->mutex = &mutex;

            std::string serialport;
            int baudrate;

            nPriv->param<std::string>("serialport", serialport, "/dev/ttyAMA0");
            nPriv->param("baudrate", baudrate, 115200);
            nPriv->param("fullRotationInSensorTicks", fullRotationInSensorTicks, 1696);
            nPriv->param("ticksPerCm", ticksPerCm, 22.836363f);

            robotWheelDistanceInCm = fullRotationInSensorTicks / ticksPerCm / M_PI; // ~23.64cm, original = 25.0cm

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
            robot->resetQueue();
            usleep(100000); // microseconds

            // Initialize the last known reference pose with a position
            // and the current values of the wheel encoders
            ROS_INFO("Computing initial reference pose");
            SensorFrame lastSensorFrame = robot->readSensorFrame();
            robot->lastKnownReferencePose = new RobotPose(0, 0, 0, robot->leftWheelDistance, robot->rightWheelDistance, ros::Time::now());

            // Topic for battery charge, capacity and light bumpers etc
            ros::Publisher sensorFrameTopic = n.advertise<::roomba500::RoombaSensorFrame>("sensorframe", 1000);

            // Here goes the odometry data
            ros::Publisher odom = n.advertise<nav_msgs::Odometry>("odom", 50);
            odomTopic = &odom;

            // Used for the tf broadcasting
            tf::TransformBroadcaster broadcaster;
            transform_broadcaster = &broadcaster;

            // We consume steering commands from here
            // ros::Subscriber cmdVelSub = n.subscribe("cmd_vel", 1000, &BaseController::newCmdVelMessage, this);

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

            // We start at the last known reference pose
            publishOdometry(robot->lastKnownReferencePose);

            // Processing the sensor polling in an endless loop until this node is shutting down
            while (ros::ok()) {

                // The main control loop. Everything is handled here
                mutex.lock();

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

                // Calculate the relative movement to last sensor data
                int deltaLeft = robot->overflowSafeWheelRotation(newSensorFrame.leftWheel - lastSensorFrame.leftWheel);
                int deltaRight = robot->overflowSafeWheelRotation(newSensorFrame.rightWheel - lastSensorFrame.rightWheel);

                ROS_DEBUG("Last wheel left = %d, last wheel right = %d, current wheel left = %d, current wheel right = %d",
                              lastSensorFrame.leftWheel, lastSensorFrame.rightWheel, newSensorFrame.leftWheel, newSensorFrame.rightWheel);

                ROS_DEBUG("Delta rotation left is %d, right is %d",
                              deltaLeft,
                              deltaRight);

                // Estimate a pose and publish information
                robot->leftWheelDistance += deltaLeft;
                robot->rightWheelDistance += deltaRight;

                ROS_DEBUG("Estimating new position");
                if (deltaLeft == 0 && deltaRight == 0) {
                    // Movement stopped, we can publish the final pose.
                    if (robot->leftWheelDistance > 0 || robot->rightWheelDistance > 0) {
                        publishFinalPose();
                    } else {
                        // We did not move, but we publish the last known reference as
                        // the current odometry
                        RobotPose* temp = new RobotPose(robot->lastKnownReferencePose, ros::Time::now());
                        delete robot->lastKnownReferencePose;
                        robot->lastKnownReferencePose = temp;

                        publishOdometry(robot->lastKnownReferencePose);
                    }

                    // Dequeue cmd_vel commands here
                    robot->dequeueCommand();
                } else {
                    // We are still moving, we will publish a pose estimation
                    delete estimateAndPublishPose();
                }

                // Remember last sensor data for the next iteration
                lastSensorFrame = newSensorFrame;

                // Publish telemetry data such as battery charge etc.
                ::roomba500::RoombaSensorFrame sensorFrameData = ::roomba500::RoombaSensorFrame();
                sensorFrameData.batteryCharge = lastSensorFrame.batteryCharge;
                sensorFrameData.batteryCapacity = lastSensorFrame.batteryCapacity;
                sensorFrameData.bumperLeft = newSensorFrame.isBumperLeft();
                sensorFrameData.bumperRight = newSensorFrame.isBumperRight();
                sensorFrameData.wheeldropLeft = newSensorFrame.isWheeldropLeft();
                sensorFrameData.wheeldropRight = newSensorFrame.isWheeldropRight();
                sensorFrameData.lightBumperLeft = lastSensorFrame.lightBumperLeft;
                sensorFrameData.lightBumperFrontLeft = lastSensorFrame.lightBumperFrontLeft;
                sensorFrameData.lightBumperCenterLeft = lastSensorFrame.lightBumperCenterLeft;
                sensorFrameData.lightBumperCenterRight = lastSensorFrame.lightBumperCenterRight;
                sensorFrameData.lightBumperFrontRight = lastSensorFrame.lightBumperFrontRight;
                sensorFrameData.lightBumperRight = lastSensorFrame.lightBumperRight;
                sensorFrameData.wheelEncoderLeft = lastSensorFrame.leftWheel;
                sensorFrameData.wheelEncoderRight = lastSensorFrame.rightWheel;

                sensorFrameTopic.publish(sensorFrameData);

                mutex.unlock();

                ros::spinOnce();
                loop_rate.sleep();
            }

            ROS_INFO("Entering passive mode");
            robot->passiveMode();

            ROS_INFO("Finishing");
            delete robot;

            ROS_INFO("BaseController terminated.");
            return 0;
        }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roomba500basecontroller");

    ros::NodeHandle nPriv("~");

    BaseController controller;

    return controller.run(&nPriv);
}