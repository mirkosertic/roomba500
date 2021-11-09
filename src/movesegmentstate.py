#!/usr/bin/env python

import rospy
import tf
import math

from basestate import BaseState
from donothingstate import DoNothingState

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveSegmentState(BaseState):

    def __init__(self, pathmanager, targetpose):
        BaseState.__init__(self, pathmanager)
        self.targetpose = targetpose

        orientation_q = targetpose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.targetYawInDegrees = self.toDegrees(yaw)
        self.counter = 0
        self.rotationSpeed = 0.0

        odomQuat = pathmanager.latestOdometry.pose.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        odomyawInDegrees = self.toDegrees(odomyaw)

        rospy.loginfo("Current odometry yaw is %s degrees", odomyawInDegrees)

        deltaYawInDegrees = self.targetYawInDegrees - odomyawInDegrees

        rospy.loginfo("Delta yaw is %s degrees", deltaYawInDegrees)

        self.targetOdomYawInDegrees = self.clampDegrees(odomyawInDegrees + deltaYawInDegrees)

        rospy.loginfo("Target odom yaw is %s degrees", self.targetOdomYawInDegrees)

        if (deltaYawInDegrees > 0):
            if (abs(deltaYawInDegrees) <= 180):
                # Rotate right
                self.rotationDirection = -1
                rospy.loginfo("Rotating right(clockwise)")
            else:
                self.rotationDirection = 1
                rospy.loginfo("Rotating left(counter-clockwise) as an optimization")
        else:
            if (abs(deltaYawInDegrees) <= 180):
                # Rotate left
                self.rotationDirection = 1
                rospy.loginfo("Rotating left(counter-clockwise)")
            else:
                self.rotationDirection = -1
                rospy.loginfo("Rotating right(clockwise) as an optimization")


    def setRotationSpeed(self, targetSpeed):
        if self.rotationSpeed != targetSpeed:
            self.pathmanager.driver.rotateZ(targetSpeed)
            self.rotationSpeed = targetSpeed


    def process(self):
        self.counter = self.counter + 1
        if (self.counter > 200):
            # We need to stop
            rospy.loginfo("Stopping robot due to timeout")

            self.pathmanager.driver.stop()

            return DoNothingState(self.pathmanager)

        odomQuat = self.pathmanager.latestOdometry.pose.pose.orientation

        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])
        odomyawInDegrees = self.toDegrees(odomyaw)

        deltaToTargetInDegrees = self.targetOdomYawInDegrees - odomyawInDegrees

        rospy.loginfo("Current odom yaw %s, target is %s, delta is %s", odomyawInDegrees, self.targetOdomYawInDegrees, deltaToTargetInDegrees)

        # We stop rotation either if we are pretty close
        # to the target yaw, or if we overshoot.
        overshot = False
        if (self.rotationSpeed == 1 and odomyawInDegrees > self.targetOdomYawInDegrees):
            overshot = True
        if (self.rotationSpeed == -1 and odomyawInDegrees < self.targetOdomYawInDegrees):
            overshot = True

        if (abs(deltaToTargetInDegrees) < 1.5 or overshot):
            # We need to stop
            rospy.loginfo("Stopping robot, as deltaToTarget = %s degrees, overshot = %s", deltaToTargetInDegrees, overshot)

            self.pathmanager.driver.stop()
            return DoNothingState(self.pathmanager)

        else:
            # Continue rotation with a reasonable speed
            targetSpeed = 0.25

            # As soon as we come close to the target angle
            # we slowdown rotation speed to make sure we do not overshoot
            if (abs(deltaToTargetInDegrees) < 25):
                targetSpeed = 0.05

            rotationSpeed = targetSpeed * self.rotationDirection

            rospy.loginfo("Continue rotation with speed = %s", targetSpeed)
            self.setRotationSpeed(rotationSpeed)

        return self


    def abort(self):
        self.pathmanager.driver.stop()

        return
