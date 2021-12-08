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
        distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

        rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s", deltaX, deltaY, distance)


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

        relativeAngle = odometryDegrees - degreesToTarget

        if (relativeAngle > 180):
            relativeAngle = 360 - relativeAngle

        if (relativeAngle < -180):
            relativeAngle = -360 - relativeAngle

        rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s, Relative Angle = %s, Odometry angle = %s, Target Angle = %s", deltaX, deltaY, distance, relativeAngle, odometryDegrees, degreesToTarget)

        # Either we are close to the target position or we overshot
        # We are at the nearest point if either the distance is near a reasonable limit
        # or the angle to the target point is equal or more than 90 degrees
        if (distance <= 0.01 or abs(relativeAngle) >= 90.0):
            # We are near the right place
            rospy.loginfo("Stopping, as we can't get nearer to the desired point")

            self.pathmanager.driver.stop()
            return self.success()

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
            if (distance > 0.15):
                # This is pretty unclear too
                rospy.loginfo("Comming closer, slowing down")
                multiplier = 0.5
            else:
                # This is pretty unclear too
                rospy.loginfo("Comming more closer, slowing down")
                multiplier = 0.25

        if (relativeAngle > 0):
            if (relativeAngle > 1):
                rospy.loginfo("Correcting heading by turning to right")
                rotationSpeed = -0.25
            else:
                rospy.loginfo("No heading correction required")

        if (relativeAngle < 0):
            if (relativeAngle < -1):
                rospy.loginfo("Correcting heading by turning to left")
                rotationSpeed = 0.25
            else:
                rospy.loginfo("No heading correction required")

        self.setDriveSpeed(driveSpeed * multiplier, rotationSpeed)

        # We continue to drive, and stay in the current system state
        return self


    def abort(self):
        self.pathmanager.driver.stop()

        return
