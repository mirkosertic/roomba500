#!/usr/bin/env python3

import rospy
import threading
import tf
import math

from donothingstate import DoNothingState
from movesegmentstate import MoveSegmentState
from rotatetoanglestate import RotateToAngleState
from driver import Driver

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Twist, Vector3

class PathManager:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.systemState = None
        self.latestOdometry = None


    def newOdomMessage(self, data):

        rospy.logdebug("Received new odometry data")

        self.syncLock.acquire()

        self.latestOdometry = data

        self.syncLock.release()
        return


    def newMoveBaseSimpleGoalMessage(self, data):

        rospy.loginfo("Received move_base_simple/goal message : %s", data)

        self.syncLock.acquire()

        self.systemState.abort()

        success = lambda previousState : DoNothingState(self, None, None)
        error = lambda previousState : DoNothingState(self, None, None)

        # First of all we orientate in the right direction
        # This is done by the RotateToAngleState
        self.systemState = RotateToAngleState(self, data, success, error)

        self.syncLock.release()
        return


    def latestOdometryTransformedToFrame(self, targetFrame):

        mpose = PoseStamped()
        mpose.pose.position = self.latestOdometry.pose.pose.position
        mpose.pose.orientation = self.latestOdometry.pose.pose.orientation
        mpose.header.frame_id = self.latestOdometry.header.frame_id
        mpose.header.stamp = self.latestOdometry.header.stamp

        latestCommonTime = self.transformlistener.getLatestCommonTime(targetFrame, self.latestOdometry.header.frame_id)
        return self.transformlistener.transformPose(targetFrame, mpose)


    def start(self):
        rospy.init_node('pathmanager', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '40'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # We consume odometry here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

        # And the driver will publish twist commands
        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size = 10))

        self.transformlistener = tf.TransformListener()

        self.systemState = DoNothingState(self, None, None)

        # Processing the sensor polling in an endless loop until this node goes to die
        while not rospy.is_shutdown():

            self.syncLock.acquire()

            self.systemState = self.systemState.process()

            self.syncLock.release()

            rate.sleep()


if __name__ == '__main__':
    try:
        manager = PathManager()
        manager.start()
    except rospy.ROSInterruptException:
        pass
