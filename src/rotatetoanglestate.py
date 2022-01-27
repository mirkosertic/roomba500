#!/usr/bin/env python

import rospy
import tf

from basestate import BaseState

class RotateToAngleState(BaseState):

    def __init__(self, pathmanager, targetposition, targetframeid, successLambda, errorLambda):
        BaseState.__init__(self, pathmanager, successLambda, errorLambda)

        rospy.loginfo("Current State : RotateToAngleState")

        self.targetPosition = targetposition
        self.targetFrameId = targetframeid
        self.rotationSpeed = None

    def setRotationSpeed(self, targetSpeed):
        if self.rotationSpeed != targetSpeed:
            self.pathmanager.driver.drive(.0, targetSpeed)
            self.rotationSpeed = targetSpeed

    def process(self):
        try:
            odomposition, odomyawInDegrees = self.currentOdometryPositionAndOrientation(self.targetFrameId)

            # This is the target movement point
            targetpositionx, targetpositiony = self.pathmanager.mapmanager.nearestNavigationPointTo(self.targetPosition)

            deltaX = targetpositionx - odomposition.x
            deltaY = targetpositiony - odomposition.y

            # We can now calculate the angle to the target position
            targetYawInDegrees = self.distanceToDegrees(deltaX, deltaY)

            shortestAngle = self.shortestAngle(odomyawInDegrees, targetYawInDegrees)

            rospy.loginfo("Current odom yaw %s, target is %s, shortest angle is %s", odomyawInDegrees, targetYawInDegrees, shortestAngle)

            if abs(shortestAngle) < 2:
                # We are coming closer
                rospy.loginfo("Stopping robot, as deltaToTarget = %s degrees", shortestAngle)

                self.pathmanager.driver.stop()
                return self.success()

            rotationSpeed = 0.15
            if abs(shortestAngle) > 15:
                rotationSpeed = 0.25
            if abs(shortestAngle) > 30:
                rotationSpeed = 0.4

            if shortestAngle > 0:
                self.setRotationSpeed(rotationSpeed)
            else:
                self.setRotationSpeed(-rotationSpeed)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
