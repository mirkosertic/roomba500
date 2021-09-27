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

syncLock = threading.Lock()

robot = None
odomTopic = None

transformBroadcaster = tf.TransformBroadcaster()

def publishOdometry(pose):
    """
        Publish odometry for a given pose
    """
    global odomTopic, transformBroadcaster, robot

    deltaTimeInSeconds = (pose.time - robot.lastKnownReferencePose.time).to_sec()
    if (deltaTimeInSeconds == 0):
        return

    vxInMetersPerSecond = (pose.x - robot.lastKnownReferencePose.x) / deltaTimeInSeconds
    vyInMetersPerSecond = (pose.y - robot.lastKnownReferencePose.y) / deltaTimeInSeconds
    vthInRadiansPerSecond = ((pose.theta - robot.lastKnownReferencePose.theta) * math.pi / 180) / deltaTimeInSeconds

    # Publish odometry
    odom = Odometry()
    odom.header.stamp = pose.time
    odom.header.frame_id = "odom"

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(pose.theta))

    odom.pose.pose = Pose(Point(pose.x, pose.y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vxInMetersPerSecond, vyInMetersPerSecond, 0), Vector3(0, 0, vthInRadiansPerSecond))

    odomTopic.publish(odom)

    transformBroadcaster.sendTransform(
        (pose.x, pose.y, 0.),
        odom_quat,
        pose.time,
        "base_link",
        "odom"
    )

    transformBroadcaster.sendTransform(
        (0., 0., 0.),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        pose.time,
        "odom",
        "map"
    )

    return


def overflowSafeWheelRotation(rotationDelta):
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


def estimateAndPublishPose():
    """
        Estimates the current pose based on the last known reference pose
        and wheel encoder values from a sensor frame
    """
    global robot, odomTopic, robot

    rospy.loginfo("Estimating new pose with leftWheelDistance = %s and rightWheelDistance = %s",
                  robot.leftWheelDistance, robot.rightWheelDistance)
    rospy.loginfo("Last known reference pose is at x = %s m and y = %s m, theta = %s degrees",
                  robot.lastKnownReferencePose.x, robot.lastKnownReferencePose.y, robot.lastKnownReferencePose.theta)

    # First case
    # Rotation on the spot
    # one wheel rotation is positive, the other is negative
    # If they sum up roughly to zero, the robot rotated on the spot
    sumOfWheelEncoders = robot.leftWheelDistance + robot.rightWheelDistance
    if sumOfWheelEncoders < 20:
        # Roomba rotated on the spot
        rotationInDegrees = robot.leftWheelDistance / (robot.fullRotationInSensorTicks / 360)

        rospy.loginfo("Assuming rotation on the spot, as sum of wheel encoders is %s. Rotation is %s degrees",
                      sumOfWheelEncoders, rotationInDegrees)

        poseestimation = RobotPose(robot.lastKnownReferencePose.theta - rotationInDegrees,
                                   robot.lastKnownReferencePose.x,
                                   robot.lastKnownReferencePose.y,
                                   robot.leftWheelDistance, robot.rightWheelDistance, rospy.Time.now())
        publishOdometry(poseestimation)
        return poseestimation

    else:
        # Second case
        # Robot moved in a straight line
        # Both wheels rotate in the same direction with roughly the same distance
        diffOfWheelEncoders = robot.leftWheelDistance - robot.rightWheelDistance
        if abs(diffOfWheelEncoders) < 20:
            # Robot moved in a straight line
            averageMovementInTicks = (robot.leftWheelDistance + robot.rightWheelDistance) / 2
            averageMovementInCm = averageMovementInTicks / robot.ticksPerCm

            deltaXInMeters = math.cos(math.radians(robot.lastKnownReferencePose.theta - 90)) * averageMovementInCm / 100
            deltaYInMeters = math.sin(math.radians(robot.lastKnownReferencePose.theta - 90)) * averageMovementInCm / 100

            rospy.loginfo("Assuming movement in straight line, as difference of wheel encoders is %s. distance is %s cm"
                          , diffOfWheelEncoders, averageMovementInCm)

            poseestimation = RobotPose(robot.lastKnownReferencePose.theta,
                                       robot.lastKnownReferencePose.x - deltaXInMeters,
                                       robot.lastKnownReferencePose.y + deltaYInMeters,
                                       robot.leftWheelDistance, robot.rightWheelDistance, rospy.Time.now())
            publishOdometry(poseestimation)
            return poseestimation

        else:
            # Third case
            # Rotation to the right. The left wheel moves further than the right wheel
            if (robot.leftWheelDistance > robot.rightWheelDistance):
                # Robot rotated to the right
                L = robot.robotWheelDistanceInCm / 100

                leftWheelDistanceInM = float(robot.leftWheelDistance) / robot.ticksPerCm / 100
                rightWheelDistanceInM = float(robot.rightWheelDistance) / robot.ticksPerCm / 100

                radiusInMeter = L / ((leftWheelDistanceInM / rightWheelDistanceInM) - 1.0)
                deltaTheta = rightWheelDistanceInM * 360 / (2.0 + math.pi * radiusInMeter)

                rospy.loginfo("Assuming rotation to the right with radius of %s m and angle of %s degrees",
                              radiusInMeter, deltaTheta)

                rotationRadiusToCenterInMeter = radiusInMeter + L / 2

                rotationPointX = robot.lastKnownReferencePose.x + math.sin(math.radians(robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter
                rotationPointY = robot.lastKnownReferencePose.y - math.cos(math.radians(robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter

                rospy.loginfo("Assuming rotation point is at x = %s, y = %s", rotationPointX, rotationPointY)

                newPositionX = rotationPointX + math.sin(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter
                newPositionY = rotationPointY + math.cos(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter

                newTheta = robot.lastKnownReferencePose.theta - deltaTheta

                rospy.loginfo("Estimated position is x = %s, y = %s, theta = %s", newPositionX, newPositionY, newTheta)

                poseestimation = RobotPose(newTheta,
                                           newPositionX,
                                           newPositionY,
                                           robot.leftWheelDistance, robot.rightWheelDistance, rospy.Time.now())
                publishOdometry(poseestimation)
                return poseestimation

            # Fourth case
            # Rotation to the left. The right wheel moves further than the left wheel
            if (robot.rightWheelDistance > robot.leftWheelDistance):
                # Robot rotated to the left
                L = robot.robotWheelDistanceInCm / 100

                leftWheelDistanceInM = float(robot.leftWheelDistance) / robot.ticksPerCm / 100
                rightWheelDistanceInM = float(robot.rightWheelDistance) / robot.ticksPerCm / 100

                radiusInMeter = L / ((rightWheelDistanceInM / leftWheelDistanceInM) - 1.0)
                deltaTheta = leftWheelDistanceInM * 360 / (2.0 + math.pi * radiusInMeter)

                rospy.loginfo("Assuming rotation to the left with radius of %s m and angle of %s degrees",
                              radiusInMeter, deltaTheta)

                rotationRadiusToCenterInMeter = radiusInMeter + L / 2

                rotationPointX = robot.lastKnownReferencePose.x + math.sin(math.radians(robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter
                rotationPointY = robot.lastKnownReferencePose.y + math.cos(math.radians(robot.lastKnownReferencePose.theta)) * rotationRadiusToCenterInMeter

                rospy.loginfo("Assuming rotation point is at x = %s, y = %s", rotationPointX, rotationPointY)

                newPositionX = rotationPointX + math.sin(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter
                newPositionY = rotationPointY - math.cos(math.radians(deltaTheta)) * rotationRadiusToCenterInMeter

                newTheta = robot.lastKnownReferencePose.theta + deltaTheta

                rospy.loginfo("Estimated position is x = %s, y = %s, theta = %s", newPositionX, newPositionY, newTheta)

                poseestimation = RobotPose(newTheta,
                                           newPositionX,
                                           newPositionY,
                                           robot.leftWheelDistance, robot.rightWheelDistance, rospy.Time.now())
                publishOdometry(poseestimation)
                return poseestimation

    # Don't know how to handle
    raise AssertionError


def publishFinalPose():
    """
        Estimates the final pose estimation based on the last known
        pose and wheel encoder values from a sensor frame. The estimated
        pose will become the new reference pose
    """
    global robot, odomTopic

    temp  = estimateAndPublishPose()

    rospy.loginfo("Final Delta rotation left is %s, right is %s",
                  overflowSafeWheelRotation(temp.leftWheel - robot.lastKnownReferencePose.leftWheel),
                  overflowSafeWheelRotation(temp.rightWheel - robot.lastKnownReferencePose.rightWheel))

    robot.lastKnownReferencePose = temp

    robot.leftWheelDistance = 0
    robot.rightWheelDistance = 0


def stopRobot():
    """
        Stops all robot motion
    """
    publishFinalPose()

    robot.drive(0, 0)


def newCmdVelMessage(data):
    """
        Consume a steering command.
        Every new steering leads to a final position estimation and
        an update to the reference pose
    """
    global odomTopic, robot, syncLock

    syncLock.acquire()

    publishFinalPose()

    rospy.loginfo("Received cmd_vel message : %s", data)

    linear = data.linear
    angular = data.angular

    speedInMeterPerSecond = linear.x # Meter per second
    rotationInRadiansPerSecond = angular.z # Radians per second

    if speedInMeterPerSecond == 0 and rotationInRadiansPerSecond == 0:

        robot.drive(0, 0)

    else:

        if rotationInRadiansPerSecond == 0:
            mmPerSecond = speedInMeterPerSecond * 100 * 10

            if mmPerSecond > 500:
                mmPerSecond = 500
            if mmPerSecond < - 500:
                mmPerSecond = -500

            robot.drive(int(mmPerSecond), int(mmPerSecond))
        else:
            degreePerSecond = rotationInRadiansPerSecond * 180.0 / math.pi

            mmPerSecond = int(robot.fullRotationInSensorTicks / 360 * degreePerSecond)
            if mmPerSecond > 500:
                mmPerSecond = 500
            if mmPerSecond < - 500:
                mmPerSecond = -500

            robot.drive(-mmPerSecond, mmPerSecond)

    syncLock.release()

def newCmdPWMMainBrush(data):
    global robot, syncLock

    syncLock.acquire()

    robot.mainbrushPWM = data.value
    robot.updateMotorControl()

    syncLock.release()

def newCmdPWMSideBrush(data):
    global robot, syncLock

    syncLock.acquire()

    robot.sidebrushPWM = data.value
    robot.updateMotorControl()

    syncLock.release()

def newCmdPWMVacuum(data):
    global robot, syncLock

    syncLock.acquire()

    robot.vacuumPWM = data.value
    robot.updateMotorControl()

    syncLock.release()

def robotmanager():
    """
        All heavy lifting of the ros node goes to here
    """
    global robot, odomTopic
    rospy.init_node('roomba500basecontroller', anonymous=True)

    # Get some configuration data
    port = rospy.get_param('~serialport', '/dev/serial0')
    baudrate = int(rospy.get_param('~baudrate', '115200'))
    fullRotationInSensorTicks = float(rospy.get_param('~fullRotationInSensorTicks', '1608.0'))
    ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.5798'))
    robotWheelDistanceInCm = float(rospy.get_param('~robotWheelDistanceInCm', '25.0'))
    pollingRateInHertz = int(rospy.get_param('~pollingintervalionhertz', '60'))

    # And connect to the roomba
    rospy.loginfo("Connecting to Roomba 5xx on port %s with %s baud", port, baudrate)
    ser = serial.Serial(port=port, baudrate=baudrate)
    robot = Roomba500(ser, fullRotationInSensorTicks, ticksPerCm, robotWheelDistanceInCm)

    # Enter safe mode
    rospy.loginfo("Entering safe mode")
    robot.safeMode()
    time.sleep(.1)

    # Signal welcome by playing a simple note
    robot.playNote(69, 16)

    # Reset all movement to zero, e.g. stop motors
    rospy.loginfo("Stopping motors")
    robot.drive(0, 0)

    # Initialize the last known reference pose with a position
    # and the current values of the wheel encoders
    rospy.loginfo("Computing initial reference pose")
    lastSensorFrame = robot.readSensorFrame()
    robot.lastKnownReferencePose = RobotPose(0, 0, 0, robot.leftWheelDistance, robot.rightWheelDistance, rospy.Time.now())

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

    # Here goes the odometry data
    odomTopic = rospy.Publisher('odom', Odometry, queue_size = 10)

    # We consume steering commands from here
    rospy.Subscriber("cmd_vel", Twist, newCmdVelMessage)

    # We also consume motor control commands
    rospy.Subscriber("cmd_mainbrush", Int16, newCmdPWMMainBrush)
    rospy.Subscriber("cmd_sidebrush", Int16, newCmdPWMSideBrush)
    rospy.Subscriber("cmd_vacuum", Int16, newCmdPWMVacuum)

    # Initialize sensor polling
    rospy.loginfo("Polling Roomba sensors with %s hertz", pollingRateInHertz)
    rate = rospy.Rate(pollingRateInHertz)

    # We start at the last known reference pose
    publishOdometry(robot.lastKnownReferencePose)

    # Processing the sensor polling in an endless loop until this node goes to die
    while not rospy.is_shutdown():

        syncLock.acquire()

        # Read some sensor data
        rospy.loginfo("Getting new sensorframe")
        newSensorFrame = robot.readSensorFrame()

        # Bumper right with debounce
        if newSensorFrame.isBumperRight():
            if not robot.lastBumperRight:
                rospy.loginfo("Right bumper triggered")
                stopRobot()

                # Note C
                robot.playNote(72, 16)

                robot.lastBumperRight = True
                bumperRightTopic.publish(1)
        else:
            if robot.lastBumperRight:
                robot.lastBumperRight = False
                bumperRightTopic.publish(0)

        # Bumper left with debounce
        if newSensorFrame.isBumperLeft():
            if not robot.lastBumperLeft:
                rospy.loginfo("Left bumper triggered")
                stopRobot()

                # Note D
                robot.playNote(74, 16)

                robot.lastBumperLeft = True
                bumperLeftTopic.publish(1)
        else:
            if robot.lastBumperLeft:
                robot.lastBumperLeft = False
                bumperLeftTopic.publish(0)

        # Right wheel drop with debounce
        if newSensorFrame.isWheeldropRight():
            if not robot.lastRightWheelDropped:
                rospy.loginfo("Right wheel dropped")
                stopRobot()

                # Note E
                robot.playNote(76, 16)

                robot.lastRightWheelDropped = True
                wheeldropRightTopic.publish(1)
        else:
            if robot.lastRightWheelDropped:
                robot.lastRightWheelDropped = False
                wheeldropRightTopic.publish(0)

        # Left wheel drop with debounce
        if newSensorFrame.isWheeldropLeft():
            if not robot.lastLeftWheelDropped:
                rospy.loginfo("Left wheel dropped")
                stopRobot()

                # Note F
                robot.playNote(77, 16)

                robot.lastLeftWheelDropped = True
                wheeldropLeftTopic.publish(1)
        else:
            if robot.lastLeftWheelDropped:
                robot.lastLeftWheelDropped = False
                wheeldropLeftTopic.publish(0)

        # Calculate the relative movement to last sensor data
        deltaLeft = overflowSafeWheelRotation(newSensorFrame.leftWheel - lastSensorFrame.leftWheel)
        deltaRight = overflowSafeWheelRotation(newSensorFrame.rightWheel - lastSensorFrame.rightWheel)
        if deltaLeft != 0 or deltaRight != 0:
            rospy.loginfo("Last wheel left = %s, last wheel right = %s, current wheel left = %s, current wheel right = %s",
                          lastSensorFrame.leftWheel, lastSensorFrame.rightWheel, newSensorFrame.leftWheel, newSensorFrame.rightWheel)

            rospy.loginfo("Delta rotation left is %s, right is %s",
                          deltaLeft,
                          deltaRight)

            # Estimate a pose and publish information
            robot.leftWheelDistance += deltaLeft
            robot.rightWheelDistance += deltaRight

            rospy.loginfo("Estimating new position")
            estimateAndPublishPose()

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

        syncLock.release()

        rate.sleep()


if __name__ == '__main__':
    try:
        robotmanager()
    except rospy.ROSInterruptException:
        pass
