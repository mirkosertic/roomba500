#!/usr/bin/env python

import rospy
import tf

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
        self.targetYaw = yaw
        self.counter = 0
        self.rotating = False

        rospy.loginfo("Target pose in map (roll,pitch,yaw) is (%s, %s, %s)", roll, pitch, yaw)

        (currentroll, currentpitch, currentyaw) = euler_from_quaternion(pathmanager.worldLatestRotation)

        rospy.loginfo("Current pose in map (roll,pitch,yaw) is (%s, %s, %s)", currentroll, currentpitch, currentyaw)

        odomQuat = pathmanager.latestOdometry.pose.orientation

        (odomroll, odompitch, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

        rospy.loginfo("Current odometry (roll,pitch,yaw) is (%s, %s, %s)", odomroll, odompitch, odomyaw)

        self.deltaYaw = yaw - currentyaw

        rospy.loginfo("Delta yaw is %s", self.deltaYaw)

        self.targetOdomYaw = odomyaw + self.deltaYaw

        rospy.loginfo("Target odom yaw is %s", self.deltaYaw)

    def process(self):
        self.counter = self.counter + 1
        if (self.counter > 30):
            # We need to stop
            rospy.loginfo("Stopping robot due to timeout")

            self.pathmanager.driver.stop()

            return DoNothingState(self.pathmanager)

        t = self.pathmanager.transformlistener.getLatestCommonTime("base_footprint", "map")
        (trans, qrot) = self.pathmanager.transformlistener.lookupTransform("base_footprint", "map", t)

        rospy.loginfo("Transform is (%s, %s)", trans, qrot)

        (roll, pitch, yaw) = euler_from_quaternion(qrot)

        rospy.loginfo("Current transform is (pose,roll,pitch,yaw) (%s, %s, %s, %s)", trans, roll, pitch, yaw)

        deltaYaw = self.targetYaw - yaw
        if (abs(deltaYaw) < 0.05):
            # We need to stop
            rospy.loginfo("Stopping robot, as deltaYaw = %s", deltaYaw)

            self.pathmanager.driver.stop()
            return DoNothingState(self.pathmanager)

        else:
            if (deltaYaw < 0):
                # Rotate right
                rospy.loginfo("Rotate right, as deltaYaw = %s", deltaYaw)

                if not self.rotating:
                    self.pathmanager.driver.rotateZ(.25)
                    self.rotating = True

            else:
                # Rotate left
                rospy.loginfo("Rotate left, as deltaYaw = %s", deltaYaw)

                if not self.rotating:
                    self.pathmanager.driver.rotateZ(-.25)
                    self.rotating = True

        return self

    def abort(self):
        self.pathmanager.driver.stop()

        return
