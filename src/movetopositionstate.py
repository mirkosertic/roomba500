#!/usr/bin/env python3

import rospy
import math
import tf

from basestate import BaseState

class MoveToPositionState(BaseState):

    def __init__(self, pathmanager, targetposition, targetframeid, successLambda, errorLambda):
        BaseState.__init__(self, pathmanager, successLambda, errorLambda)

        rospy.loginfo("Current State : MoveToPositionState")

        self.targetFrameId = targetframeid

        self.driveSpeed = 0.0
        self.driveAngularMoment = 0.0

        # This is the target movement point
        nearestcell = self.pathmanager.map.nearestCellCovering(targetposition)
        self.targetpositionx = nearestcell.centerx
        self.targetpositiony = nearestcell.centery

        # Now, we need to get the current position
        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(targetframeid)
        currentPosition = odometryInTargetposeFrame.pose.position

        # We calculate the distance to the target position
        deltaX = self.targetpositionx - currentPosition.x
        deltaY = self.targetpositiony - currentPosition.y
        distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

    def setDriveSpeed(self, targetSpeed, targetAngularMoment):
        if self.driveSpeed != targetSpeed or self.driveAngularMoment != targetAngularMoment:
            rospy.loginfo("Setting linear velocity = %s, angular velocity = %s", targetSpeed, targetAngularMoment)
            self.pathmanager.driver.drive(targetSpeed, targetAngularMoment)
            self.driveSpeed = targetSpeed
            self.driveAngularMoment = targetAngularMoment

    def process(self):
        try:
            odomposition, odomyawInDegrees = self.currentOdometryPositionAndOrientation(self.targetFrameId)

            # We calculate the current distance to the target position
            deltaX = self.targetpositionx - odomposition.x
            deltaY = self.targetpositiony - odomposition.y
            distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)
            degreesToTarget = self.distanceToDegrees(deltaX, deltaY)

            shortestAngle = self.shortestAngle(odomyawInDegrees, degreesToTarget)

            rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s, shortest angle = %s, Odometry angle = %s, Target Angle = %s", deltaX, deltaY, distance, shortestAngle, odomyawInDegrees, degreesToTarget)

            self.pathmanager.publishNavigationInfo(distance, shortestAngle)

            # Either we are close to the target position or we overshot
            # We are at the nearest point if either the distance is near a reasonable limit
            # or the angle to the target point is equal or more than 90 degrees
            if (distance <= self.pathmanager.linear_tolerance or abs(shortestAngle) >= 90.0):
                # We are near the right place
                rospy.loginfo("Stopping, as we can't get nearer to the desired point")

                self.pathmanager.driver.stop()
                return self.success()

            (vel_x, vel_theta) = self.compute_velocity((odomposition.x, odomposition.y, odomyawInDegrees * math.pi / 180), (self.targetpositionx, self.targetpositiony, degreesToTarget * math.pi / 180))
            self.setDriveSpeed(vel_x, vel_theta)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # We continue to drive, and stay in the current system state
        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
