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

        # First of all, we need to translate the current odometry
        # to the targetpose frame to calculate the correct yaw delta
        odometryInTargetposeFrame = pathmanager.latestOdometryTransformedToFrame(targetpose.header.frame_id)

        # We calculate the target yaw
        orientation_q = targetpose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.targetYawInDegrees = self.toDegrees(yaw)
        self.counter = 0
        self.rotationSpeed = 0.0
        self.lastDeltaToTargetInDegrees = None

        rospy.loginfo("Command to rotate to %s degrees in reference frame %s", self.targetYawInDegrees, targetpose.header.frame_id)

        # Now we calculate the current odom yaw in target frame coordinate system
        odomQuat = odometryInTargetposeFrame.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        odomyawInDegrees = self.toDegrees(odomyaw)

        rospy.loginfo("Current odometry yaw is %s degrees in reference frame", odomyawInDegrees)

        deltaYawInDegrees = self.targetYawInDegrees - odomyawInDegrees

        rospy.loginfo("Delta yaw is %s degrees", deltaYawInDegrees)

        self.targetOdomYawInDegrees = self.clampDegrees(odomyawInDegrees + deltaYawInDegrees)

        rospy.loginfo("Target odom yaw is %s degrees in reference frame", self.targetOdomYawInDegrees)

        if (deltaYawInDegrees > 0):
            if (abs(deltaYawInDegrees) > 180):
                # Rotate right
                self.rotationDirection = -1
                rospy.loginfo("Rotating right(clockwise) as an optimization")
            else:
                self.rotationDirection = 1
                rospy.loginfo("Rotating left(counter-clockwise)")
        else:
            if (abs(deltaYawInDegrees) > 180):
                # Rotate left
                self.rotationDirection = 1
                rospy.loginfo("Rotating left(counter-clockwise) as an optimization")
            else:
                self.rotationDirection = -1
                rospy.loginfo("Rotating right(clockwise)")


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

        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(self.targetpose.header.frame_id)
        odomQuat = odometryInTargetposeFrame.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])
        odomyawInDegrees = self.toDegrees(odomyaw)

        deltaToTargetInDegrees = self.targetOdomYawInDegrees - odomyawInDegrees

        rospy.loginfo("Current odom yaw %s, target is %s, delta is %s, rotationDirection is %s", odomyawInDegrees, self.targetOdomYawInDegrees, deltaToTargetInDegrees, self.rotationDirection)

        # We stop rotation either if we are pretty close
        # to the target yaw, or if we overshoot.
        overshot = False
        if (self.lastDeltaToTargetInDegrees != None):
            if (self.lastDeltaToTargetInDegrees > 0 and deltaToTargetInDegrees < 0 and self.rotationDirection == 1):
                overshot = True
            if (self.lastDeltaToTargetInDegrees < 0 and deltaToTargetInDegrees > 0 and self.rotationDirection == -1):
                overshot = True

        self.lastDeltaToTargetInDegrees = deltaToTargetInDegrees

        if (abs(deltaToTargetInDegrees) < 0.5 or overshot):
            # We need to stop
            rospy.loginfo("Stopping robot, as deltaToTarget = %s degrees, overshot = %s", deltaToTargetInDegrees, overshot)

            self.pathmanager.driver.stop()
            return DoNothingState(self.pathmanager)

        else:
            # Continue rotation with a reasonable speed
            targetSpeed = 0.25

            # As soon as we come close to the target angle
            # we slowdown rotation speed to make sure we do not overshoot
            if (abs(deltaToTargetInDegrees) < 33):
                targetSpeed = 0.05

            rotationSpeed = targetSpeed * self.rotationDirection

            rospy.loginfo("Continue rotation with speed = %s", targetSpeed)
            self.setRotationSpeed(rotationSpeed)

        return self


    def abort(self):
        self.pathmanager.driver.stop()

        return
