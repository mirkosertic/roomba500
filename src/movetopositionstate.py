#!/usr/bin/env python

import rospy
import math

from basestate import BaseState

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveToPositionState(BaseState):

    def __init__(self, pathmanager, targetpose, successLambda, errorLambda):
        BaseState.__init__(self, pathmanager, successLambda, errorLambda)

        rospy.loginfo("Current State : MoveToPositionState")

        self.targetPose = targetpose
        self.counter = 0
        self.driveSpeed = 0.0
        self.driveAngularMoment = 0.0
        self.lastDeltaToTargetInDegrees = None

        # This is the target movement point
        targetPosition = targetpose.pose.position

        # Now, we need to get the current position
        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(targetpose.header.frame_id)
        currentPosition = odometryInTargetposeFrame.pose.position

        # We calculate the distance to the target position
        deltaX = targetPosition.x - currentPosition.x
        deltaY = targetPosition.y - currentPosition.y
        self.lastDistance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

        rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s", deltaX, deltaY, self.lastDistance)


    def setDriveSpeed(self, targetSpeed, targetAngularMoment):
        if self.driveSpeed != targetSpeed or self.driveAngularMoment != targetAngularMoment:
            self.pathmanager.driver.drive(targetSpeed, targetAngularMoment)
            self.driveSpeed = targetSpeed
            self.driveAngularMoment = targetAngularMoment


    def process(self):
        self.counter = self.counter + 1
        if (self.counter > 200):
            # We need to stop
            rospy.loginfo("Stopping robot due to timeout")

            self.pathmanager.driver.stop()

            return self.error()

        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(self.targetPose.header.frame_id)
        currentPosition = odometryInTargetposeFrame.pose.position

        # We calculate the current distance to the target position
        deltaX = self.targetPose.pose.position.x - currentPosition.x
        deltaY = self.targetPose.pose.position.y - currentPosition.y
        distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)
        degreesToTarget = self.distanceToDegrees(deltaX, deltaY)

        # And we check the current alignment angle using odometry to verify we
        # are heading in the right direction
        odomQuat = odometryInTargetposeFrame.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        odometryDegrees = self.toDegrees(odomyaw)

        angleDelta = degreesToTarget - odometryDegrees

        rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s, Angle to Target = %s, Odometry Angle = %s, Delta Angle = %s", deltaX, deltaY, distance, degreesToTarget, odometryDegrees, angleDelta)

        # Either we are close to the target position or we overshot (distance is larger than last known distance)
        deltaDistance = self.lastDistance - distance
        if (deltaDistance < 0.05 or distance > self.lastDistance):
            # We are near the right place
            rospy.loginfo("Stopping due to delta = %s and last distance = %s", deltaDistance, self.lastDistance)

            self.pathmanager.driver.stop()
            return self.success()

        if (deltaDistance > 0.30):
            # Guess what is happening here
            rospy.loginfo("Far away, using more speed")
            self.setDriveSpeed(0.20, .0)
        else:
            # This is pretty unclear too
            rospy.loginfo("Comming closer, slowing down")
            self.setDriveSpeed(0.10, .0)

        self.lastDistance = distance

        # We continue to drive, and stay in the current system state
        return self


    def abort(self):
        self.pathmanager.driver.stop()

        return
