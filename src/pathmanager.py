#!/usr/bin/env python3

import rospy
import threading
import tf

from donothingstate import DoNothingState
from rotatetoanglestate import RotateToAngleState
from movetopositionstate import MoveToPositionState
from driver import Driver
from mapmanager import MapManager

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from visualization_msgs.msg import MarkerArray


class PathManager:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.systemState = None
        self.latestOdometry = None
        self.doNothingLambda = lambda previousState: DoNothingState(self, None, None)
        self.errorLambda = lambda previousState: DoNothingState(self, None, None)
        self.mapmanager = None
        self.driver = None
        self.transformlistener = None

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

        odometryInTargetposeFrame = self.latestOdometryTransformedToFrame(data.header.frame_id)

        currentposition = self.mapmanager.nearestNavigationPointTo(odometryInTargetposeFrame.pose.position)
        targetposition = self.mapmanager.nearestNavigationPointTo(data.pose.position)

        path = self.mapmanager.findPath(currentposition, targetposition)
        if path is not None:

            # Draw some debug information
            points = []
            points.append(currentposition)
            for pos in path:
                points.append(pos)

            self.mapmanager.highlightPath(points)

            rospy.loginfo("Path to target is : %s", path)

            #
            # Calculate the next waypoint
            #
            def goToPositionState(pathmanager, path, position):
                if position >= len(path):
                    rospy.loginfo("Reached end of path")
                    return DoNothingState(self, None, None)

                nextstate = lambda prevstate: goToPositionState(pathmanager, path, position + 1)
                afterotationstate = lambda prevstate: MoveToPositionState(pathmanager, Point(x, y, 0), data.header.frame_id, nextstate, self.errorLambda)

                (x, y) = path[position]
                rospy.loginfo("Going to next waypoint #%s, (%s, %s)", position, x, y)
                return RotateToAngleState(pathmanager, Point(x, y, 0), data.header.frame_id, afterotationstate, self.errorLambda)

            nextstate = lambda prevstate: goToPositionState(self, path, 0)

            # First we orientate in the right direction
            # This is done by the RotateToAngleState
            (x, y) = path[0]
            self.systemState = RotateToAngleState(self, Point(x, y, 0), data.header.frame_id, nextstate, self.errorLambda)

        self.syncLock.release()
        return

    def latestOdometryTransformedToFrame(self, targetframe):

        mpose = PoseStamped()
        mpose.pose.position = self.latestOdometry.pose.pose.position
        mpose.pose.orientation = self.latestOdometry.pose.pose.orientation
        mpose.header.frame_id = self.latestOdometry.header.frame_id
        mpose.header.stamp = self.latestOdometry.header.stamp

        latestcommontime = self.transformlistener.getLatestCommonTime(targetframe, self.latestOdometry.header.frame_id)
        return self.transformlistener.transformPose(targetframe, mpose)

    def start(self):
        rospy.init_node('pathmanager', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '40'))

        debugimagelocation = rospy.get_param('~debugimagelocation', None)

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # We consume odometry, maps and move base goals here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

        # This is our map manager, responsible for
        # navigation, pathfinding and visualization
        self.mapmanager = MapManager(rospy.Publisher('visualization_marker', MarkerArray, queue_size=10), debugimagelocation)
        rospy.Subscriber("map", OccupancyGrid, self.mapmanager.newMapData)

        # And the driver will publish twist commands
        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

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
