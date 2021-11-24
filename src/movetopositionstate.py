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
            rospy.loginfo("Setting linear velocity = %s, angular velovity = %s", targetSpeed, targetAngularMoment)
            self.pathmanager.driver.drive(targetSpeed, targetAngularMoment)
            self.driveSpeed = targetSpeed
            self.driveAngularMoment = targetAngularMoment


    def process(self):
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
        if (distance < 0.02 or (self.lastDistance - distance > 0.02)):
            # We are near the right place
            rospy.loginfo("Stopping due to distance = %s, last distance = %s", distance, self.lastDistance)

            self.pathmanager.driver.stop()
            return self.success()

        if (abs(angleDelta) > 20):
            # There is something wrong with our heading, we need to realign
            # TODO: Implement realign
            rospy.loginfo("Stopping due angle misalignment with delta = %s", angleDelta)

            self.pathmanager.driver.stop()
            return self.error()

        #
        # Positive Angle Delta -> Rotate left, else right
        #

        driveSpeed = 0.35
        rotationSpeed = .0
        multiplier = 1.0

        if (distance > 0.30):
            # Guess what is happening here
            rospy.loginfo("Far away, using more speed")
            multiplier = 1
        else:
            # This is pretty unclear too
            rospy.loginfo("Comming closer, slowing down")
            multiplier = 0.5

        if (angleDelta > 0):
            if (angleDelta > 1):
                rospy.loginfo("Correcting heading by turning to left")
                rotationSpeed = 0.25
            else:
                rospy.loginfo("No heading correction required")
        if (angleDelta < 0):
            if (angleDelta < 1):
                rospy.loginfo("Correcting heading by turning to right")
                rotationSpeed = -0.25
            else:
                rospy.loginfo("No heading correction required")

        self.setDriveSpeed(driveSpeed * multiplier, rotationSpeed)

        self.lastDistance = distance

        # We continue to drive, and stay in the current system state
        return self


    def abort(self):
        self.pathmanager.driver.stop()

        return
