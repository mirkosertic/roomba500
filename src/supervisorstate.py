#!/usr/bin/env python3

import threading

import tf

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class SupervisorState:

    def __init__(self, transformlistener, mapframe):
        self.syncLock = threading.Lock()
        self.robotnode = None
        self.latestbatterycharge = None
        self.latestbatterycapacity = None
        self.bumperleft = False
        self.bumperright = False
        self.wheeldropleft = False
        self.wheeldropright = False
        self.lightbumperleft = 0
        self.lightbumperfrontleft = 0
        self.lightbumpercenterleft = 0
        self.lightbumpercenterright = 0
        self.lightbumperfrontright = 0
        self.lightbumperright = 0
        self.odometrylog = None
        self.lastcommand = ''
        self.amclmode = False

        self.mapframe = mapframe
        self.transformlistener = transformlistener

        self.latestpositiononmap = None
        self.latestyawonmap = None
        pass

    def newSensorFrame(self, message):
        self.syncLock.acquire()
        self.latestbatterycharge = message.batteryCharge.data
        self.latestbatterycapacity = message.batteryCapacity.data
        self.bumperleft = message.bumperLeft
        self.bumperright = message.bumperRight
        self.wheeldropleft = message.wheeldropLeft
        self.wheeldropright = message.wheeldropRight
        self.lightbumperleft = message.lightBumperLeft.data
        self.lightbumperfrontleft = message.lightBumperFrontLeft.data
        self.lightbumpercenterleft = message.lightBumperCenterLeft.data
        self.lightbumpercenterright = message.lightBumperCenterRight.data
        self.lightbumperfrontright = message.lightBumperFrontRight.data
        self.lightbumperright = message.lightBumperRight.data
        self.syncLock.release()
        pass

    def initOdometryLog(self, file):
        self.odometrylog = file
        separator = ','
        self.odometrylog.write('event' + separator + 'timestamp' + separator + 'positionx' + separator + 'positiony' + separator + 'rotationz' + separator + 'velocityx' + separator + 'velocityy' + separator + 'velocityz\n')
        self.odometrylog.flush()

    def closeOdometryLog(self):
        self.odometrylog.close()
        self.odometrylog = None

    def newOdometry(self, message):
        position = message.pose.pose.position
        orientation_q = message.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        velocity = message.twist.twist

        if self.odometrylog is not None:
            separator = ','
            self.odometrylog.write(self.lastcommand + separator + str(message.header.stamp) + separator + '{:.2f}'.format(position.x) + separator + '{:.2f}'.format(position.y) + separator + '{:.2f}'.format(yaw) + separator + '{:.2f}'.format(velocity.linear.x) + separator + '{:.2f}'.format(velocity.linear.y) + separator + '{:.2f}'.format(velocity.angular.z) + '\n')
            self.odometrylog.flush()

        self.lastcommand = ''

        try:
            mpose = PoseStamped()
            mpose.pose.position = message.pose.pose.position
            mpose.pose.orientation = message.pose.pose.orientation
            mpose.header.frame_id = message.header.frame_id
            mpose.header.stamp = message.header.stamp

            latestcommontime = self.transformlistener.getLatestCommonTime(self.mapframe, message.header.frame_id)
            odometryInTargetposeFrame = self.transformlistener.transformPose(self.mapframe, mpose)

            odomQuat = odometryInTargetposeFrame.pose.orientation
            (_, _, odomyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

            self.latestpositiononmap = odometryInTargetposeFrame.pose.position
            self.latestyawonmap = odomyaw

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # Do nothing here
            pass

        pass


    def gathersystemstate(self):
        data = {
            'awake': self.robotnode is not None,
            'batteryCapacity': self.latestbatterycapacity,
            'batteryCharge': self.latestbatterycharge,
            'bumperLeft': self.bumperleft,
            'bumperRight': self.bumperright,
            'wheeldropLeft': self.wheeldropleft,
            'wheeldropRight': self.wheeldropright,
            'lightbumperLeft': self.lightbumperleft,
            'lightbumperFrontLeft': self.lightbumperfrontleft,
            'lightbumperCenterLeft': self.lightbumpercenterleft,
            'lightbumperCenterRight': self.lightbumpercenterright,
            'lightbumperFrontRight': self.lightbumperfrontright,
            'lightbumperRight': self.lightbumperright,
            'amclmode': self.amclmode,
        }
        return data
