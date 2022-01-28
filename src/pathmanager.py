#!/usr/bin/env python3

import rospy
import threading
import tf

from donothingstate import DoNothingState
from rotatetoanglestate import RotateToAngleState
from movetopositionstate import MoveToPositionState
from driver import Driver
from mapmanager import MapManager

from std_msgs.msg import Int16
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Vector3
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

    def newMoveBaseSimpleGoalMessage(self, data):

        rospy.loginfo("Received move_base_simple/goal message : %s", data)

        self.syncLock.acquire()

        self.systemState.abort()

        odometryInTargetposeFrame = self.latestOdometryTransformedToFrame(data.header.frame_id)

        currentposition = self.mapmanager.nearestNavigationPointTo(odometryInTargetposeFrame.pose.position)
        targetposition = self.mapmanager.nearestNavigationPointTo(data.pose.position)

        path = self.mapmanager.findPath(currentposition, targetposition)
        if path is not None and len(path) > 0:

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
        else:
            rospy.loginfo("Cannot follow path, as there is no path or path is too short : %s", str(path))

        self.syncLock.release()

    def newShutdownCommand(self, data):

        rospy.logdebug("Received shutdown command")

        self.syncLock.acquire()

        rospy.signal_shutdown('Shutdown requested')

        self.syncLock.release()

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
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        debugimagelocation = rospy.get_param('~debugimagelocation', None)

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # We consume odometry, maps and move base goals here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

        # This is our map manager, responsible for
        # navigation, pathfinding and visualization
        self.mapmanager = MapManager(rospy.Publisher('visualization_marker', MarkerArray, queue_size=10, latch=True), debugimagelocation)
        rospy.Subscriber("map", OccupancyGrid, self.mapmanager.newMapData)

        # And the driver will publish twist commands
        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        self.transformlistener = tf.TransformListener()

        self.systemState = DoNothingState(self, None, None)

        # Check if we should init the map by calling the map_server instead of
        # waiting for a published map. This is the case if we are running AMCL instead of SLAM
        if rospy.get_param('~initfrommapserver', False):
            servicename = rospy.get_param('~servicename', 'static_map')

            rospy.loginfo('Waiting for service %s', servicename)
            rospy.wait_for_service(servicename)
            rospy.loginfo('Invoking service')
            service = rospy.ServiceProxy(servicename, GetMap)
            response = service()
            rospy.loginfo('Ready to rumble!')

            self.mapmanager.newMapData(response.map)

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            self.syncLock.acquire()

            self.systemState = self.systemState.process()

            self.syncLock.release()

            rate.sleep()

        rospy.loginfo('Pathmanager terminated.')

if __name__ == '__main__':
    try:
        manager = PathManager()
        manager.start()
    except rospy.ROSInterruptException:
        pass
