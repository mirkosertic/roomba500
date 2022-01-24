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
        self.bumperleft = 0
        self.bumperright = 0
        self.wheeldropleft = 0
        self.wheeldropright = 0
        self.lightbumperleft = 0
        self.lightbumperfrontleft = 0
        self.lightbumpercenterleft = 0
        self.lightbumpercenterright = 0
        self.lightbumperfrontright = 0
        self.lightbumperright = 0
        self.odometrylog = None
        self.lastcommand = ''

        self.mapframe = mapframe
        self.transformlistener = transformlistener

        self.latestpositiononmap = None
        self.latestyawonmap = None
        pass

    def newBatteryCapacity(self, message):
        self.syncLock.acquire()
        self.latestbatterycapacity = message.data
        self.syncLock.release()

    def newBatteryCharge(self, message):
        self.syncLock.acquire()
        self.latestbatterycharge = message.data
        self.syncLock.release()

    def newBumperLeft(self, message):
        self.syncLock.acquire()
        self.bumperleft = message.data
        self.syncLock.release()

    def newBumperRight(self, message):
        self.syncLock.acquire()
        self.bumperright = message.data
        self.syncLock.release()

    def newWheeldropLeft(self, message):
        self.syncLock.acquire()
        self.wheeldropleft = message.data
        self.syncLock.release()

    def newWheeldropRight(self, message):
        self.syncLock.acquire()
        self.wheeldropright = message.data
        self.syncLock.release()

    def newLightBumperLeft(self, message):
        self.syncLock.acquire()
        self.lightbumperleft = message.data
        self.syncLock.release()

    def newLightBumperFrontLeft(self, message):
        self.syncLock.acquire()
        self.lightbumperfrontleft = message.data
        self.syncLock.release()

    def newLightBumperCenterLeft(self, message):
        self.syncLock.acquire()
        self.lightbumpercenterleft = message.data
        self.syncLock.release()

    def newLightBumperCenterRight(self, message):
        self.syncLock.acquire()
        self.lightbumpercenterright = message.data
        self.syncLock.release()

    def newLightBumperFrontRight(self, message):
        self.syncLock.acquire()
        self.lightbumperfrontright = message.data
        self.syncLock.release()

    def newLightBumperRight(self, message):
        self.syncLock.acquire()
        self.lightbumperright = message.data
        self.syncLock.release()

    def initOdometryLog(self, file):
        self.odometrylog = file
        separator = ','
        self.odometrylog.write('event' + separator + 'timestamp' + separator + 'positionx' + separator + 'positiony' + separator + 'rotationz' + separator + 'velocityx' + separator + 'velocityy' + separator + 'velocityz\n')
        self.odometrylog.flush()

    def newOdometry(self, message):
        position = message.pose.pose.position
        orientation_q = message.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        velocity = message.twist.twist

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
            'bumperLeft': True if self.bumperleft > 0 else False,
            'bumperRight': True if self.bumperright > 0 else False,
            'wheeldropLeft': True if self.wheeldropleft > 0 else False,
            'wheeldropRight': True if self.wheeldropright > 0 else False,
            'lightbumperLeft': self.lightbumperleft,
            'lightbumperFrontLeft': self.lightbumperfrontleft,
            'lightbumperCenterLeft': self.lightbumpercenterleft,
            'lightbumperCenterRight': self.lightbumpercenterright,
            'lightbumperFrontRight': self.lightbumperfrontright,
            'lightbumperRight': self.lightbumperright,
        }
        return data
