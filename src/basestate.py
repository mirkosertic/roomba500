#!/usr/bin/env python3

import math
from tf.transformations import euler_from_quaternion

class BaseState:

    def __init__(self, pathmanager, successLambda, errorLambda):
        self.pathmanager = pathmanager
        self.successLambda = successLambda
        self.errorLambda = errorLambda
        return

    def success(self):
        return self.successLambda(self)

    def error(self):
        return self.errorLambda(self)

    def process(self):
        return self

    def abort(self):
        return

    def clampDegrees(self, value):
        while (value < 0):
            value += 360
        while (value >= 360):
            value -= 360

        return value

    def toDegrees(self, value):
        return self.clampDegrees(math.degrees(value))

    def distanceToDegrees(self, dx, dy):
        return self.toDegrees(math.atan2(dy, dx))

    def shortestAngle(self, origin, target):
        # Code taken from https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles/28037434
        diff = (target - origin + 180) % 360 - 180
        if diff < -180:
            return diff + 360
        return diff

    def currentOdometryPositionAndOrientation(self, targetframe):
        odometryInTargetposeFrame = self.pathmanager.latestOdometryTransformedToFrame(targetframe)
        odomQuat = odometryInTargetposeFrame.pose.orientation
        (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])
        odomyawInDegrees = self.toDegrees(odomyaw)
        return odometryInTargetposeFrame.pose.position, odomyawInDegrees

    def normalize_pi(self, alpha):
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        return alpha

    def compute_velocity(self, currentPose, targetPose):
        # Code taken from https://github.com/merose/diff_drive/blob/master/src/diff_drive/goal_controller.py
        (cur_x, cur_y, cur_theta) = currentPose
        (goal_x, goal_y, goal_theta) = targetPose

        goal_heading = math.atan2(goal_y - cur_y, goal_x - cur_x)
        a = -cur_theta + goal_heading

        theta = self.normalize_pi(cur_theta - goal_theta)
        b = -theta - a

        diffx = goal_x - cur_x
        diffy = goal_y - cur_y

        d = math.sqrt(diffx * diffx + diffy * diffy)
        a = self.normalize_pi(a)
        b = self.normalize_pi(b)

        if abs(d) < self.pathmanager.linear_tolerance:
            desired_xvel = .0
            desired_thetavel = self.pathmanager.kB * theta
        else:
            desired_xvel = self.pathmanager.kP * d
            desired_thetavel = self.pathmanager.kA * a + self.pathmanager.kB * b

        # Adjust velocities if X velocity is too high.
        if abs(desired_xvel) > self.pathmanager.max_linear_speed:
            ratio = self.pathmanager.max_linear_speed / abs(desired_xvel)
            desired_xvel *= ratio
            desired_thetavel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired_thetavel) > self.pathmanager.max_angular_speed:
            ratio = self.pathmanager.max_angular_speed / abs(desired_thetavel)
            desired_xvel *= ratio
            desired_thetavel *= ratio

        if abs(desired_xvel) > 0 and abs(desired_xvel) < self.pathmanager.min_linear_speed:
            ratio = self.pathmanager.min_linear_speed / abs(desired_xvel)
            desired_xvel *= ratio
            desired_thetavel *= ratio
        elif desired_xvel==0 and abs(desired_thetavel) < self.pathmanager.min_angular_speed:
            ratio = self.pathmanager.min_angular_speed / abs(desired_thetavel)
            desired_xvel *= ratio
            desired_thetavel *= ratio

        return (desired_xvel, desired_thetavel)
