#!/usr/bin/env python3
import math

import rospy
import logging
import traceback

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
            nearestcell = self.pathmanager.map.nearestCellCovering(self.targetPosition)
            targetpositionx = nearestcell.centerx
            targetpositiony = nearestcell.centery

            deltaX = targetpositionx - odomposition.x
            deltaY = targetpositiony - odomposition.y
            distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

            # We can now calculate the angle to the target position
            targetYawInDegrees = self.distanceToDegrees(deltaX, deltaY)

            shortestAngle = self.shortestAngle(odomyawInDegrees, targetYawInDegrees)

            rospy.loginfo("Current odom yaw %s, target is %s, shortest angle is %s", odomyawInDegrees, targetYawInDegrees, shortestAngle)

            self.pathmanager.publishNavigationInfo(distance, shortestAngle)

            if abs(shortestAngle) < self.pathmanager.angular_tolerance:
                # We are coming closer
                rospy.loginfo("Stopping robot, as deltaToTarget = %s degrees", shortestAngle)

                self.pathmanager.driver.stop()

                return self.success()

            (vel_x, vel_theta) = self.compute_velocity((odomposition.x, odomposition.y, odomyawInDegrees * math.pi / 180), (odomposition.x, odomposition.y, targetYawInDegrees * math.pi / 180))
            self.setRotationSpeed(vel_theta)

        except Exception as e:
            rospy.logerr('Error calculating path : %s', e)
            logging.error(traceback.format_exc())
            pass

        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
