#!/usr/bin/env python

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
        self.lastDeltaToTargetInDegrees = None

        # This is the target movement point
        self.targetpositionx, self.targetpositiony = pathmanager.mapmanager.nearestNavigationPointTo(targetposition)

        # Now, we need to get the current position
        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(targetframeid)
        currentPosition = odometryInTargetposeFrame.pose.position

        # We calculate the distance to the target position
        deltaX = self.targetpositionx - currentPosition.x
        deltaY = self.targetpositiony - currentPosition.y
        distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

        rospy.loginfo("DeltaX = %s, DeltaY = %s, Distance = %s", deltaX, deltaY, distance)

    def setDriveSpeed(self, targetSpeed, targetAngularMoment):
        if self.driveSpeed != targetSpeed or self.driveAngularMoment != targetAngularMoment:
            rospy.loginfo("Setting linear velocity = %s, angular velovity = %s", targetSpeed, targetAngularMoment)
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

            # Either we are close to the target position or we overshot
            # We are at the nearest point if either the distance is near a reasonable limit
            # or the angle to the target point is equal or more than 90 degrees
            if (distance <= 0.01 or abs(shortestAngle) >= 90.0):
                # We are near the right place
                rospy.loginfo("Stopping, as we can't get nearer to the desired point")

                self.pathmanager.driver.stop()
                return self.success()

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

            if shortestAngle > 1:
                rospy.loginfo("Correcting heading by turning to the left")
                rotationSpeed = 0.25
            if shortestAngle < -1:
                rospy.loginfo("Correcting heading by turning to the right")
                rotationSpeed = -0.25

            self.setDriveSpeed(driveSpeed * multiplier, rotationSpeed)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # We continue to drive, and stay in the current system state
        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
