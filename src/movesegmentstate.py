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
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.targetRoll = roll
        self.targetPitch = pitch
        self.targetYaw = self.clampRadians(yaw)
        self.counter = 10
        self.rotationSpeed = 0.0

        odomQuat = pathmanager.latestOdometry.pose.pose.orientation

        (odomroll, odompitch, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        rospy.loginfo("Current odometry (roll,pitch,yaw) is (%s, %s, %s)", odomroll, odompitch, odomyaw)

        self.deltaYaw = self.targetYaw - odomyaw

        rospy.loginfo("Delta yaw is %s", self.deltaYaw)

        self.targetOdomYaw = self.clampRadians(odomyaw + self.deltaYaw)

        rospy.loginfo("Target odom yaw is %s", self.deltaYaw)

    def clampRadians(self, value):
        doublePi = math.pi * 2
        if (value > doublePi):
            value = value - doublePi
        if (value < -doublePi):
            value = value + doublePi

        return value

    def setRotationSpeed(self, targetSpeed):
        if self.rotationSpeed != targetSpeed:
            self.pathmanager.driver.rotateZ(targetSpeed)
            self.rotationSpeed = targetSpeed

    def process(self):
        self.counter = self.counter + 1
        if (self.counter > 500):
            # We need to stop
            rospy.loginfo("Stopping robot due to timeout")

            self.pathmanager.driver.stop()

            return DoNothingState(self.pathmanager)

        odomQuat = self.pathmanager.latestOdometry.pose.pose.orientation

        (odomroll, odompitch, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        rospy.loginfo("Current odom yaw %s, target is %s, delta is %s", odomyaw, self.targetOdomYaw, (self.targetOdomYaw - odomyaw))

        if (abs(odomyaw - self.targetOdomYaw) < 0.03):
            # We need to stop
            rospy.loginfo("Stopping robot, as deltaYaw = %s", (odomyaw - self.targetOdomYaw))

            self.pathmanager.driver.stop()
            return DoNothingState(self.pathmanager)

        else:
            if (self.deltaYaw < 0):
                # Rotate right
                targetSpeed = -0.25
                if (abs(odomyaw - self.targetOdomYaw) < 0.25):
                    targetSpeed = -0.05

                rospy.loginfo("Rotate right, as deltaYaw = %s, speed is %s", self.deltaYaw, targetSpeed)

                self.setRotationSpeed(targetSpeed)

            else:
                # Rotate left
                targetSpeed = 0.25
                if (abs(odomyaw - self.targetOdomYaw) < 0.25):
                    targetSpeed = 0.05

                rospy.loginfo("Rotate left, as deltaYaw = %s, speed is %s", self.deltaYaw, targetSpeed)

                self.setRotationSpeed(targetSpeed)

        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
