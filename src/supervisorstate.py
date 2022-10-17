#!/usr/bin/env python3

import threading
import os

import tf

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class SupervisorState:

    def __init__(self, transformlistener, mapframe, roomsdirectory):
        self.syncLock = threading.Lock()
        self.robotnode = None
        self.ready = False
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
        self.wheelEncoderLeft = 0
        self.wheelEncoderRight = 0
        self.lightbumperLeftStat = False
        self.lightbumperFrontLeftStat = False
        self.lightbumperCenterLeftStat = False
        self.lightbumperCenterRightStat = False
        self.lightbumperFrontRightStat = False
        self.lightbumperRightStat = False
        self.distanceToTargetInMeters = .0
        self.angleToTargetInDegrees = .0
        self.currentWaypoint = 0
        self.numWaypoints = 0

        self.mapframe = mapframe
        self.transformlistener = transformlistener

        self.latestodomonmap = None

        self.lastcommandedvelx = .0
        self.lastcommandedveltheta = .0

        self.roomsdirectory = roomsdirectory
        self.latestcleaningpath = None
        self.latestcleaningmap = None

        self.loggingqueues = []
        self.mapqueues = []
        self.costmapqueues = []
        self.statequeues = []

        self.knownservices = []

    def newCleaningPath(self, message):

        pending = []

        for pose in message.poses:
            odomQuat = pose.pose.orientation
            (_, _, theta) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

            pending.append({
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'theta': theta
            })

        data = {
            'pending': pending
        }
        self.latestcleaningpath = data

    def newCleaningMap(self, message):

        self.latestcleaningmap = message

    def newMap(self, message):

        data = {
            'width': message.info.width,
            'height': message.info.height,
            'resolution': message.info.resolution,
            'data': message.data,
            'origin': {
                'x': message.info.origin.position.x,
                'y': message.info.origin.position.y
            }
        }
        for q in self.mapqueues:
            q.put_nowait(data)

    def newCostMap(self, message):

        data = {
            'width': message.info.width,
            'height': message.info.height,
            'resolution': message.info.resolution,
            'data': message.data,
            'origin': {
                'x': message.info.origin.position.x,
                'y': message.info.origin.position.y
            }
        }
        for q in self.costmapqueues:
            q.put_nowait(data)

    def newLogMessage(self, message):
        data = {
            'stamp': str(message.header.stamp),
            'level': message.level,
            'node': message.name,
            'msg': message.msg
        }
        for q in self.loggingqueues:
            q.put_nowait(data)

    def newSensorFrame(self, message):
        self.syncLock.acquire()

        self.latestbatterycharge = message.batteryCharge
        self.latestbatterycapacity = message.batteryCapacity
        self.bumperleft = message.bumperLeft
        self.bumperright = message.bumperRight
        self.wheeldropleft = message.wheeldropLeft
        self.wheeldropright = message.wheeldropRight
        self.lightbumperleft = message.lightBumperLeft
        self.lightbumperfrontleft = message.lightBumperFrontLeft
        self.lightbumpercenterleft = message.lightBumperCenterLeft
        self.lightbumpercenterright = message.lightBumperCenterRight
        self.lightbumperfrontright = message.lightBumperFrontRight
        self.lightbumperright = message.lightBumperRight
        self.wheelEncoderLeft = message.wheelEncoderLeft
        self.wheelEncoderRight = message.wheelEncoderRight
        self.lightbumperLeftStat = message.lightBumperLeftStat
        self.lightbumperFrontLeftStat = message.lightBumperFrontLeftStat
        self.lightbumperCenterLeftStat = message.lightBumperCenterLeftStat
        self.lightbumperCenterRightStat = message.lightBumperCenterRightStat
        self.lightbumperFrontRightStat = message.lightBumperFrontRightStat
        self.lightbumperRightStat = message.lightBumperRightStat

        self.syncLock.release()

    def newOdometry(self, message):
        try:
            self.syncLock.acquire()

            latestcommontime = self.transformlistener.getLatestCommonTime(self.mapframe, message.header.frame_id)

            mpose = PoseStamped()
            mpose.pose.position = message.pose.pose.position
            mpose.pose.orientation = message.pose.pose.orientation
            mpose.header.frame_id = message.header.frame_id
            mpose.header.stamp = latestcommontime

            odometryInTargetposeFrame = self.transformlistener.transformPose(self.mapframe, mpose)

            odomQuat = odometryInTargetposeFrame.pose.orientation
            (_, _, theta) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

            self.latestodomonmap = {
                'x': odometryInTargetposeFrame.pose.position.x,
                'y': odometryInTargetposeFrame.pose.position.y,
                'theta': theta,
                'velx': message.twist.twist.linear.x,
                'veltheta': message.twist.twist.angular.z
            }

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # Do nothing here
            pass

        self.syncLock.release()

    def newCmdVel(self, message):
        self.syncLock.acquire()

        self.lastcommandedvelx = message.linear.x
        self.lastcommandedveltheta = message.angular.z

        self.syncLock.release()

    def newNavigationInfo(self, message):
        self.syncLock.acquire()

        self.distanceToTargetInMeters = message.distanceToTargetInMeters
        self.angleToTargetInDegrees = message.angleToTargetInDegrees
        self.currentWaypoint = message.currentWaypoint
        self.numWaypoints = message.numWaypoints

        self.syncLock.release()

    def gathersystemstate(self):

        knownrooms = next(os.walk(self.roomsdirectory))[1]

        data = {
            'awake': self.robotnode is not None and self.ready,
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
            'wheelEncoderLeft': self.wheelEncoderLeft,
            'wheelEncoderRight': self.wheelEncoderRight,
            'lastcommandedvelx': self.lastcommandedvelx,
            'lastcommandedveltheta': self.lastcommandedveltheta,
            'odomvelx': self.latestodomonmap['velx'] if self.latestodomonmap is not None else .0,
            'odomveltheta': self.latestodomonmap['veltheta'] if self.latestodomonmap is not None else .0,
            'distanceToTargetInMeters': self.distanceToTargetInMeters,
            'angleToTargetInDegrees': self.angleToTargetInDegrees,
            'currentWaypoint': self.currentWaypoint,
            'numWaypoints': self.numWaypoints,
            'lightbumperLeftStat': self.lightbumperLeftStat,
            'lightbumperFrontLeftStat': self.lightbumperFrontLeftStat,
            'lightbumperCenterLeftStat': self.lightbumperCenterLeftStat,
            'lightbumperCenterRightStat': self.lightbumperCenterRightStat,
            'lightbumperFrontRightStat': self.lightbumperFrontRightStat,
            'lightbumperRightStat': self.lightbumperRightStat,
            'hasrelocalization': '/global_localization' in self.knownservices,
            'hasclean': '/clean' in self.knownservices,
            'hascancel': '/cancel' in self.knownservices,
            'odometryOnMap': self.latestodomonmap,
            'cleaningpath': self.latestcleaningpath,
            'cleaningmap': self.latestcleaningmap,
            'rooms': knownrooms
        }
        return data
