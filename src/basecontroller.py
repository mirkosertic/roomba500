#!/usr/bin/env python

import serial
import time
import rospy
import tf
import math
import threading

from std_msgs.msg import String, Int16
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3

from roomba500 import Roomba500
from robotpose import RobotPose

class BaseController:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.robot = None
        self.odomTopic = None
        self.transformBroadcaster = tf.TransformBroadcaster()

    def publishOdometry(self, pose):
        """
            Publish odometry for a given pose
        """
        deltaTimeInNanoSeconds = (pose.time - self.robot.lastKnownReferencePose.time).to_nsec()
        if deltaTimeInNanoSeconds == 0:
            return

        # The robot can only move forward ( x - direction in base_link coordinate frame
        distanceX = pose.x - self.robot.lastKnownReferencePose.x
        distanceY = pose.y - self.robot.lastKnownReferencePose.y
        linearDistanceInMeters = math.sqrt(distanceX * distanceX + distanceY * distanceY)

        vxInMetersPerSecond = linearDistanceInMeters / deltaTimeInNanoSeconds * 1000000000
        vyInMetersPerSecond = .0
        vthInRadiansPerSecond = -((pose.theta - self.robot.lastKnownReferencePose.theta) * math.pi / 180) / deltaTimeInNanoSeconds * 1000000000

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = pose.time
        odom.header.frame_id = "odom"

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(pose.theta))

        odom.pose.pose = Pose(Point(pose.x, pose.y, 0.), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vxInMetersPerSecond, vyInMetersPerSecond, 0), Vector3(0, 0, vthInRadiansPerSecond))

        self.transformBroadcaster.sendTransform(
            (pose.x, pose.y, 0.),
            odom_quat,
            pose.time,
            "base_link",
            "odom"
        )

        self.odomTopic.publish(odom)

        return


    def overflowSafeWheelRotation(self, rotationDelta):
        """
            Check if the delta wheel rotation was caused by some overflow
        """
        if rotationDelta < -16384:
            # Rotation forward with overflow
            return rotationDelta + 65536
        if rotationDelta > 16384:
            # Rotation backward with overflow
            return rotationDelta - 65536

        return rotationDelta


    def estimateAndPublishPose(self):
        """
            Estimates the current pose based on the last known reference pose
            and wheel encoder values from a sensor frame
        """
        rospy.logdebug("Estimating new pose with leftWheelDistance = %s and rightWheelDistance = %s",
                      self.robot.leftWheelDistance, self.robot.rightWheelDistance)
        rospy.logdebug("Last known reference pose is at x = %s m and y = %s m, theta = %s degrees",
                      self.robot.lastKnownReferencePose.x, self.robot.lastKnownReferencePose.y, self.robot.lastKnownReferencePose.theta)

        # First case
        # Rotation on the spot
        # one wheel rotation is positive, the other is negative
        # If they sum up roughly to zero, the robot rotated on the spot
        sumOfWheelEncoders = self.robot.leftWheelDistance + self.robot.rightWheelDistance
        if sumOfWheelEncoders < 20:
            # Roomba rotated on the spot
            rotationInDegrees = self.robot.leftWheelDistance / (self.robot.fullRotationInSensorTicks / 360)

            rospy.logdebug("Assuming rotation on the spot, as sum of wheel encoders is %s. Rotation is %s degrees",
                          sumOfWheelEncoders, rotationInDegrees)

            poseestimation = RobotPose(self.robot.lastKnownReferencePose.theta - rotationInDegrees,
                                       self.robot.lastKnownReferencePose.x,
                                       self.robot.lastKnownReferencePose.y,
                                       self.robot.leftWheelDistance, self.robot.rightWheelDistance, rospy.Time.now())
            self.publishOdometry(poseestimation)
            return poseestimation

        else:
            # Second case
            # Robot moved in a straight line
            # Both wheels rotate in the same direction with roughly the same distance
            diffOfWheelEncoders = self.robot.leftWheelDistance - self.robot.rightWheelDistance
            if abs(diffOfWheelEncoders) < 20:
                # Robot moved in a straight line
                averageMovementInTicks = (self.robot.leftWheelDistance + self.robot.rightWheelDistance) / 2.0
                averageMovementInCm = averageMovementInTicks / self.robot.ticksPerCm

                deltaXInMeters = math.cos(math.radians(self.robot.lastKnownReferencePose.theta)) * averageMovementInCm / 100
                deltaYInMeters = math.sin(math.radians(self.robot.lastKnownReferencePose.theta)) * averageMovementInCm / 100

                rospy.logdebug("Assuming movement in straight line, as difference of wheel encoders is %s. distance is %s cm, dx = %s meters, dy = %s meters"
                              , diffOfWheelEncoders, averageMovementInCm, deltaXInMeters, deltaYInMeters)

                poseestimation = RobotPose(self.robot.lastKnownReferencePose.theta,
                                           self.robot.lastKnownReferencePose.x + deltaXInMeters,
                                           self.robot.lastKnownReferencePose.y + deltaYInMeters,
                                           self.robot.leftWheelDistance, self.robot.rightWheelDistance, rospy.Time.now())
                self.publishOdometry(poseestimation)
                return poseestimation

            else:
                # Third case
                # Rotation to the right. The left wheel moves further than the right wheel
                if (self.robot.leftWheelDistance > self.robot.rightWheelDistance):
                    # Robot rotated to the right
                    L = self.robot.robotWheelDistanceInCm / 100

                    leftWheelDistanceInM = float(self.robot.leftWheelDistance) / self.robot.ticksPerCm / 100
                    rightWheelDistanceInM = float(self.robot.rightWheelDistance) / self.robot.ticksPerCm / 100

                    radiusInMeter = L / ((leftWheelDistanceInM / rightWheelDistanceInM) - 1.0)
                    deltaTheta = rightWheelDistanceInM * 360 / (2.0 + math.pi * radiusInMeter)

                    rospy.logdebug("Assuming rotation to the right with radius of %s m and angle of %s degrees",
                                  radiusInMeter, deltaTheta)

                    rotationRadiusToCenterInMeter = radiusInMeter + L / 2

                    rotationPointX = self.robot.lastKnownReferencePose.x + math.sin(math.radians(self.robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter
                    rotationPointY = self.robot.lastKnownReferencePose.y - math.cos(math.radians(self.robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter

                    rospy.logdebug("Assuming rotation point is at x = %s, y = %s", rotationPointX, rotationPointY)

                    newPositionX = rotationPointX + math.sin(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter
                    newPositionY = rotationPointY + math.cos(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter

                    newTheta = self.robot.lastKnownReferencePose.theta - deltaTheta

                    rospy.logdebug("Estimated position is x = %s, y = %s, theta = %s", newPositionX, newPositionY, newTheta)

                    poseestimation = RobotPose(newTheta,
                                               newPositionX,
                                               newPositionY,
                                               self.robot.leftWheelDistance, self.robot.rightWheelDistance, rospy.Time.now())
                    self.publishOdometry(poseestimation)
                    return poseestimation

                # Fourth case
                # Rotation to the left. The right wheel moves further than the left wheel
                if (self.robot.rightWheelDistance > self.robot.leftWheelDistance):
                    # Robot rotated to the left
                    L = self.robot.robotWheelDistanceInCm / 100

                    leftWheelDistanceInM = float(self.robot.leftWheelDistance) / self.robot.ticksPerCm / 100
                    rightWheelDistanceInM = float(self.robot.rightWheelDistance) / self.robot.ticksPerCm / 100

                    radiusInMeter = L / ((rightWheelDistanceInM / leftWheelDistanceInM) - 1.0)
                    deltaTheta = leftWheelDistanceInM * 360 / (2.0 + math.pi * radiusInMeter)

                    rospy.logdebug("Assuming rotation to the left with radius of %s m and angle of %s degrees",
                                  radiusInMeter, deltaTheta)

                    rotationRadiusToCenterInMeter = radiusInMeter + L / 2

                    rotationPointX = self.robot.lastKnownReferencePose.x + math.sin(math.radians(self.robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter
                    rotationPointY = self.robot.lastKnownReferencePose.y + math.cos(math.radians(self.robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter

                    rospy.logdebug("Assuming rotation point is at x = %s, y = %s", rotationPointX, rotationPointY)

                    newPositionX = rotationPointX + math.sin(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter
                    newPositionY = rotationPointY - math.cos(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter

                    newTheta = self.robot.lastKnownReferencePose.theta + deltaTheta

                    rospy.logdebug("Estimated position is x = %s, y = %s, theta = %s", newPositionX, newPositionY, newTheta)

                    poseestimation = RobotPose(newTheta,
                                               newPositionX,
                                               newPositionY,
                                               self.robot.leftWheelDistance, self.robot.rightWheelDistance, rospy.Time.now())
                    self.publishOdometry(poseestimation)
                    return poseestimation

        # Don't know how to handle
        raise AssertionError


    def publishFinalPose(self):
        """
            Estimates the final pose estimation based on the last known
            pose and wheel encoder values from a sensor frame. The estimated
            pose will become the new reference pose
        """
        temp  = self.estimateAndPublishPose()

        rospy.logdebug("Final Delta rotation left is %s, right is %s",
                      self.overflowSafeWheelRotation(temp.leftWheel - self.robot.lastKnownReferencePose.leftWheel),
                      self.overflowSafeWheelRotation(temp.rightWheel - self.robot.lastKnownReferencePose.rightWheel))

        self.robot.lastKnownReferencePose = temp

        self.robot.leftWheelDistance = 0
        self.robot.rightWheelDistance = 0


    def stopRobot(self):
        """
            Stops all robot motion
        """
        self.publishFinalPose()

        self.robot.drive(0, 0)


    def newCmdVelMessage(self, data):
        """
            Consume a steering command.
            Every new steering leads to a final position estimation and
            an update to the reference pose
        """
        self.syncLock.acquire()

        self.publishFinalPose()

        rospy.logdebug("Received cmd_vel message : %s", data)

        linear = data.linear
        angular = data.angular

        speedInMeterPerSecond = linear.x # Meter per second
        rotationInRadiansPerSecond = angular.z # Radians per second

        if speedInMeterPerSecond == 0.0 and rotationInRadiansPerSecond == 0.0:

            self.robot.drive(0, 0)

        else:

            if rotationInRadiansPerSecond == 0.0:
                mmPerSecond = speedInMeterPerSecond * 100 * 10

                if mmPerSecond > 500:
                    mmPerSecond = 500
                if mmPerSecond < - 500:
                    mmPerSecond = -500

                self.robot.drive(int(mmPerSecond), int(mmPerSecond))
            else:
                degreePerSecond = rotationInRadiansPerSecond * 180.0 / math.pi

                mmPerSecond = int(self.robot.fullRotationInSensorTicks / 360 * degreePerSecond)
                if mmPerSecond > 500:
                    mmPerSecond = 500
                if mmPerSecond < - 500:
                    mmPerSecond = -500

                self.robot.drive(-mmPerSecond, mmPerSecond)

        self.syncLock.release()

    def newCmdPWMMainBrush(self, data):
        self.syncLock.acquire()

        self.robot.mainbrushPWM = data.data
        self.robot.updateMotorControl()

        self.syncLock.release()

    def newCmdPWMSideBrush(self, data):
        self.syncLock.acquire()

        self.robot.sidebrushPWM = data.data
        self.robot.updateMotorControl()

        self.syncLock.release()

    def newCmdPWMVacuum(self, data):
        self.syncLock.acquire()

        rospy.logdebug("Received vacuum pwm command %s", data)

        self.robot.vacuumPWM = data.data
        self.robot.updateMotorControl()

        self.syncLock.release()

    def start(self):
        """
            All heavy lifting of the ros node goes to here
        """
        rospy.init_node('roomba500basecontroller', anonymous=True)

        # Get some configuration data
        port = rospy.get_param('~serialport', '/dev/serial0')
        baudrate = int(rospy.get_param('~baudrate', '115200'))
        fullRotationInSensorTicks = float(rospy.get_param('~fullRotationInSensorTicks', '1608.0'))
        ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.5798'))
        robotWheelDistanceInCm = float(rospy.get_param('~robotWheelDistanceInCm', '25.0'))
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '30'))

        # And connect to the roomba
        rospy.loginfo("Connecting to Roomba 5xx on port %s with %s baud", port, baudrate)
        ser = serial.Serial(port=port, baudrate=baudrate)
        self.robot = Roomba500(ser, fullRotationInSensorTicks, ticksPerCm, robotWheelDistanceInCm)

        # Enter safe mode
        rospy.loginfo("Entering safe mode")
        self.robot.safeMode()
        time.sleep(.1)

        # Signal welcome by playing a simple note
        self.robot.playNote(69, 16)

        # Reset all movement to zero, e.g. stop motors
        rospy.loginfo("Stopping motors")
        self.robot.drive(0, 0)

        # Initialize the last known reference pose with a position
        # and the current values of the wheel encoders
        rospy.loginfo("Computing initial reference pose")
        lastSensorFrame = self.robot.readSensorFrame()
        self.robot.lastKnownReferencePose = RobotPose(0, 0, 0, self.robot.leftWheelDistance, self.robot.rightWheelDistance, rospy.Time.now())

        # Topics for battery charge, capacity and light bumpers
        batteryChargeTopic = rospy.Publisher('batteryCharge', Int16, queue_size = 10)
        batteryCapacityTopic = rospy.Publisher('batteryCapacity', Int16, queue_size = 10)
        bumperLeftTopic = rospy.Publisher('bumperLeft', Int16, queue_size = 10)
        bumperRightTopic = rospy.Publisher('bumperRight', Int16, queue_size = 10)
        wheeldropLeftTopic = rospy.Publisher('wheeldropLeft', Int16, queue_size = 10)
        wheeldropRightTopic = rospy.Publisher('wheeldropRight', Int16, queue_size = 10)

        lightBumperLeftTopic = rospy.Publisher('lightBumperLeft', Int16, queue_size = 10)
        lightBumperFrontLeftTopic = rospy.Publisher('lightBumperFrontLeft', Int16, queue_size = 10)
        lightBumperCenterLeftTopic = rospy.Publisher('lightBumperCenterLeft', Int16, queue_size = 10)
        lightBumperCenterRightTopic = rospy.Publisher('lightBumperCenterRight', Int16, queue_size = 10)
        lightBumperFrontRightTopic = rospy.Publisher('lightBumperFrontRight', Int16, queue_size = 10)
        lightBumperRightTopic = rospy.Publisher('lightBumperRight', Int16, queue_size = 10)

        oimodeTopic = rospy.Publisher('oimode', Int16, queue_size = 10)

        # Here goes the odometry data
        self.odomTopic = rospy.Publisher('odom', Odometry, queue_size = 10)

        # We consume steering commands from here
        rospy.Subscriber("cmd_vel", Twist, self.newCmdVelMessage)

        # We also consume motor control commands
        rospy.Subscriber("cmd_mainbrush", Int16, self.newCmdPWMMainBrush)
        rospy.Subscriber("cmd_sidebrush", Int16, self.newCmdPWMSideBrush)
        rospy.Subscriber("cmd_vacuum", Int16, self.newCmdPWMVacuum)

        # Initialize sensor polling
        rospy.loginfo("Polling Roomba sensors with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # We start at the last known reference pose
        self.publishOdometry(self.robot.lastKnownReferencePose)

        # Processing the sensor polling in an endless loop until this node goes to die
        while not rospy.is_shutdown():

            self.syncLock.acquire()

            # Read some sensor data
            rospy.loginfo("Getting new sensorframe")
            newSensorFrame = self.robot.readSensorFrame()

            # Bumper right with debounce
            if newSensorFrame.isBumperRight():
                if not self.robot.lastBumperRight:
                    rospy.logdebug("Right bumper triggered")
                    self.stopRobot()

                    # Note C
                    self.robot.playNote(72, 16)

                    self.robot.lastBumperRight = True
                    bumperRightTopic.publish(1)
            else:
                if self.robot.lastBumperRight:
                    self.robot.lastBumperRight = False
                    bumperRightTopic.publish(0)

            # Bumper left with debounce
            if newSensorFrame.isBumperLeft():
                if not self.robot.lastBumperLeft:
                    rospy.logdebug("Left bumper triggered")
                    self.stopRobot()

                    # Note D
                    self.robot.playNote(74, 16)

                    self.robot.lastBumperLeft = True
                    bumperLeftTopic.publish(1)
            else:
                if self.robot.lastBumperLeft:
                    self.robot.lastBumperLeft = False
                    bumperLeftTopic.publish(0)

            # Right wheel drop with debounce
            if newSensorFrame.isWheeldropRight():
                if not self.robot.lastRightWheelDropped:
                    rospy.logdebug("Right wheel dropped")
                    self.stopRobot()

                    # Note E
                    self.robot.playNote(76, 16)

                    self.robot.lastRightWheelDropped = True
                    wheeldropRightTopic.publish(1)
            else:
                if self.robot.lastRightWheelDropped:
                    self.robot.lastRightWheelDropped = False
                    wheeldropRightTopic.publish(0)

            # Left wheel drop with debounce
            if newSensorFrame.isWheeldropLeft():
                if not self.robot.lastLeftWheelDropped:
                    rospy.logdebug("Left wheel dropped")
                    self.stopRobot()

                    # Note F
                    self.robot.playNote(77, 16)

                    self.robot.lastLeftWheelDropped = True
                    wheeldropLeftTopic.publish(1)
            else:
                if self.robot.lastLeftWheelDropped:
                    self.robot.lastLeftWheelDropped = False
                    wheeldropLeftTopic.publish(0)

            # Calculate the relative movement to last sensor data
            deltaLeft = self.overflowSafeWheelRotation(newSensorFrame.leftWheel - lastSensorFrame.leftWheel)
            deltaRight = self.overflowSafeWheelRotation(newSensorFrame.rightWheel - lastSensorFrame.rightWheel)

            rospy.logdebug("Last wheel left = %s, last wheel right = %s, current wheel left = %s, current wheel right = %s",
                          lastSensorFrame.leftWheel, lastSensorFrame.rightWheel, newSensorFrame.leftWheel, newSensorFrame.rightWheel)

            rospy.logdebug("Delta rotation left is %s, right is %s",
                          deltaLeft,
                          deltaRight)

            # Estimate a pose and publish information
            self.robot.leftWheelDistance += deltaLeft
            self.robot.rightWheelDistance += deltaRight

            rospy.logdebug("Estimating new position")
            self.estimateAndPublishPose()

            # Remember last sensor data for the next iteration
            lastSensorFrame = newSensorFrame

            # Publish telemetry data such as battery charge etc.
            batteryChargeTopic.publish(lastSensorFrame.batteryCharge)
            batteryCapacityTopic.publish(lastSensorFrame.batteryCapacity)

            lightBumperLeftTopic.publish(lastSensorFrame.lightBumperLeft)
            lightBumperFrontLeftTopic.publish(lastSensorFrame.lightBumperFrontLeft)
            lightBumperCenterLeftTopic.publish(lastSensorFrame.lightBumperCenterLeft)
            lightBumperCenterRightTopic.publish(lastSensorFrame.lightBumperCenterRight)
            lightBumperFrontRightTopic.publish(lastSensorFrame.lightBumperFrontRight)
            lightBumperRightTopic.publish(lastSensorFrame.lightBumperRight)

            oimodeTopic.publish(lastSensorFrame.oimode)

            self.syncLock.release()

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = BaseController()
        controller.start()
    except rospy.ROSInterruptException:
        pass
