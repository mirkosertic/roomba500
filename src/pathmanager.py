#!/usr/bin/env python3

import rospy
import threading
import tf

from donothingstate import DoNothingState
from movesegmentstate import MoveSegmentState
from driver import Driver

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3, PoseStamped

class PathManager:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.systemState = None
        self.latestOdometry = None

    def newOdomMessage(self, data):
        self.syncLock.acquire()

        rospy.logdebug("Received new odometry data")

        self.latestOdometry = data

        self.syncLock.release()
        return

    def newMoveBaseSimpleGoalMessage(self, data):

        rospy.logdebug("Received move_base_simple/goal message : %s", data)

        self.syncLock.acquire()

        self.systemState.abort()
        self.systemState = MoveSegmentState(self, data)

        self.syncLock.release()
        return

    def start(self):
        rospy.init_node('pathmanager', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # We consume odometry here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

        # And the driver will publish twist commands
        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size = 10))

        self.transformlistener = tf.TransformListener()

        self.systemState = DoNothingState(self)

        # Processing the sensor polling in an endless loop until this node goes to die
        while not rospy.is_shutdown():

            self.syncLock.acquire()

            #t = self.transformlistener.getLatestCommonTime("base_footprint", "map")
            #(transformation, rotation) = self.transformlistener.lookupTransform("base_link", "map", t)

            #self.worldLatestTime = t
            #self.worldLatestTransformation = transformation
            #self.worldLatestRotation = rotation
            #self.worldLastX = transformation[0]
            #self.worldLastY = transformation[1]
            #self.worldLastZ = transformation[2]

            self.systemState = self.systemState.process()

            self.syncLock.release()

            rate.sleep()

if __name__ == '__main__':
    try:
        manager = PathManager()
        manager.start()
    except rospy.ROSInterruptException:
        pass
