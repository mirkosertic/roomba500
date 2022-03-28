#!/usr/bin/env python3

import cv2
import rospy
import threading
import tf
import logging
import traceback

from donothingstate import DoNothingState
from rotatetoanglestate import RotateToAngleState
from movetopositionstate import MoveToPositionState
from driver import Driver
from map import NavigationMap, GridCellStatus

from std_msgs.msg import Int16, ColorRGBA
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

from roomba500.msg import NavigationInfo
from roomba500.srv import Clean, CleanResponse, Cancel, CancelResponse


class PathManager:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.systemState = None
        self.latestOdometry = None
        self.doNothingLambda = lambda previousState: DoNothingState(self, None, None)
        self.errorLambda = lambda previousState: DoNothingState(self, None, None)
        self.map = None
        self.driver = None
        self.transformlistener = None
        self.infotopic = None
        self.markerstopic = None

        self.kP = 3.0
        self.kA = 8
        self.kB = -1.5
        self.max_linear_speed = 0.2
        self.min_linear_speed = 0.08
        self.max_angular_speed = 0.8
        self.min_angular_speed = 0.3
        self.linear_tolerance = 0.025

        self.angular_tolerance = 3.0

    def newOdomMessage(self, data):

        rospy.logdebug("Received new odometry data")

        self.syncLock.acquire()

        self.latestOdometry = data

        self.syncLock.release()

    def newMoveBaseSimpleGoalMessage(self, data):

        try:
            rospy.loginfo("Received move_base_simple/goal message : %s", data)

            self.syncLock.acquire()

            self.systemState.abort()

            odometryInTargetposeFrame = self.latestOdometryTransformedToFrame(data.header.frame_id)

            currentposition = self.map.nearestCellCovering(odometryInTargetposeFrame.pose.position)
            if currentposition is None:
                rospy.loginfo("Cannot find cell for current position on map. No path finding possible")
                return

            targetposition = self.map.nearestCellCovering(data.pose.position)
            if targetposition is None:
                rospy.loginfo("Cannot find cell for target position on map. No path finding possible")
                return

            rospy.loginfo("Trying to find new path...")
            path = self.map.findPath(currentposition, targetposition)
            if path is not None and len(path) > 0:
                rospy.loginfo("There is a viable path...")
                self.followPath(path, data.header.frame_id)
            else:
                rospy.loginfo("Cannot follow path, as there is no path or path is too short : %s", str(path))

        except Exception as e:
            rospy.logerr('Error saving map : %s', e)
            logging.error(traceback.format_exc())
            pass

        self.syncLock.release()

    def followPath(self, path, frameid):

        # Draw some debug information
        self.highlightPath(path)

        rospy.loginfo("Path to target is : %s", path)

        #
        # Calculate the next waypoint
        #
        def goToPositionState(pathmanager, path, position):
            if position >= len(path):
                rospy.loginfo("Reached end of path")
                return DoNothingState(self, None, None)

            nextstate = lambda prevstate: goToPositionState(pathmanager, path, position + 1)

            afterotationstate = lambda prevstate: MoveToPositionState(pathmanager, Point(x, y, 0), frameid, nextstate, self.errorLambda)

            x = path[position].centerx
            y = path[position].centery
            rospy.loginfo("Going to next waypoint #%s, (%s, %s)", position, x, y)
            return RotateToAngleState(pathmanager, Point(x, y, 0), frameid, afterotationstate, self.errorLambda)

        # We ignore path index 0 as this is the start position
        nextstate = lambda prevstate: goToPositionState(self, path, 1)

        # First we orientate in the right direction
        # This is done by the RotateToAngleState
        x = path[1].centerx
        y = path[1].centery

        self.systemState = RotateToAngleState(self, Point(x, y, 0), frameid, nextstate, self.errorLambda)

    def newShutdownCommand(self, data):

        rospy.loginfo("Received shutdown command")

        self.syncLock.acquire()

        rospy.signal_shutdown('Shutdown requested')

        self.syncLock.release()

    def clean(self, req):
        rospy.loginfo("Received clean command")

        self.syncLock.acquire()

        odometryInTargetposeFrame = self.latestOdometryTransformedToFrame('map')
        currentposition = self.map.nearestCellCovering(odometryInTargetposeFrame.pose.position)
        if currentposition is not None:
            rospy.loginfo("Calculating full coverage path...")
            path = self.map.startCoverageBoustrophedon(currentposition)
            rospy.loginfo("Calculation finished")
            if path is not None:
                self.followPath(path, 'map')

        self.syncLock.release()

        return CleanResponse(0)

    def cancel(self, req):
        rospy.loginfo("Received cancel command")

        self.syncLock.acquire()

        self.systemState = DoNothingState(self, None, None)

        self.syncLock.release()

        return CancelResponse(0)

    def latestOdometryTransformedToFrame(self, targetframe):

        latestcommontime = self.transformlistener.getLatestCommonTime(targetframe, self.latestOdometry.header.frame_id)

        mpose = PoseStamped()
        mpose.pose.position = self.latestOdometry.pose.pose.position
        mpose.pose.orientation = self.latestOdometry.pose.pose.orientation
        mpose.header.frame_id = self.latestOdometry.header.frame_id
        mpose.header.stamp = latestcommontime

        return self.transformlistener.transformPose(targetframe, mpose)

    def publishNavigationInfo(self, distanceToTargetInMeters, angleToTargetInDegrees):
        message = NavigationInfo()
        message.distanceToTargetInMeters = distanceToTargetInMeters
        message.angleToTargetInDegrees = angleToTargetInDegrees
        self.infotopic.publish(message)

    def highlightPath(self, path):

        markers = MarkerArray()
        quat = quaternion_from_euler(0, 0, 0)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = 3
        marker.type = 4  # Line-strip
        marker.action = 0
        marker.scale.x = 0.05
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.points = []
        marker.colors = []

        for index, cell in enumerate(path):
            marker.points.append(Point(cell.centerx, cell.centery, 0.05))
            marker.colors.append(ColorRGBA(0, 0, 1, 1))

            p = Marker()
            p.header.frame_id = 'map'
            p.header.stamp = rospy.Time.now()
            p.id = 100 + index
            p.type = 2  # sphere
            p.action = 0
            p.scale.x = 0.15
            p.scale.y = 0.15
            p.scale.z = 0.15
            p.pose.position.x = cell.centerx
            p.pose.position.y = cell.centery
            p.pose.position.z = 0.0
            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]
            p.points = []
            p.colors = []
            p.color.a = 1
            p.color.r = 0
            p.color.g = 0
            p.color.b = 1

            markers.markers.append(p)


        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1

        markers.markers.append(marker)

        self.markerstopic.publish(markers)

    def publishMapState(self):
        markers = MarkerArray()

        for cell in self.map.cells:

            if cell.status == GridCellStatus.OCCUPIED or cell.status == GridCellStatus.FREE:
                quat = quaternion_from_euler(0, 0, 0)
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = rospy.Time.now()
                marker.id = int(cell.centerx * 100000000 + cell.centery * 100)
                marker.type = 1
                marker.action = 0
                marker.scale.x = self.map.scanwidthinmeters
                marker.scale.y = self.map.scanwidthinmeters
                marker.scale.z = self.map.scanwidthinmeters
                marker.pose.position.x = cell.centerx
                marker.pose.position.y = cell.centery
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                marker.color.a = 0.25

                if cell.status == GridCellStatus.FREE:
                    marker.color.r = 0
                    marker.color.g = 1
                    marker.color.b = 0
                elif cell.status == GridCellStatus.OCCUPIED:
                    marker.color.r = 1
                    marker.color.g = 0
                    marker.color.b = 0

                markers.markers.append(marker)

        self.markerstopic.publish(markers)

    def start(self):
        rospy.init_node('pathmanager', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '5'))
        statePublishIntervalInSeconds = int(rospy.get_param('~statePublishIntervalInSeconds', '2'))

        debugimagelocation = rospy.get_param('~debugimagelocation', None)

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.markerstopic = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10, latch=True)

        # This is our map responsible for navigation and pathfinding
        gridcellwidthinmeters = float(rospy.get_param('~gridcellwidthinmeters', '0.18'))
        scanwidthinmeters = float(rospy.get_param('~scanwidthinmeters', '0.36'))
        occupancythreshold = float(rospy.get_param('~occupancythreshold', ' 0.65'))
        self.map = NavigationMap(gridcellwidthinmeters, scanwidthinmeters, occupancythreshold)

        rospy.Subscriber("map", OccupancyGrid, self.map.initOrUpdateFromOccupancyGrid)

        # And the driver will publish twist commands
        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        self.transformlistener = tf.TransformListener()

        self.infotopic = rospy.Publisher('navigation_info', NavigationInfo, queue_size=10)

        self.systemState = DoNothingState(self, None, None)

        # We consume odometry, maps and move base goals here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

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

            self.map.initOrUpdateFromOccupancyGrid(response.map)

        # Register services
        rospy.Service("clean", Clean, self.clean)
        rospy.Service("cancel", Cancel, self.cancel)

        # Processing the sensor polling in an endless loop until this node shuts down
        cyclecounter = 0
        while not rospy.is_shutdown():

            self.syncLock.acquire()

            self.systemState = self.systemState.process()

            # Publish debug state
            cyclecounter = cyclecounter + 1
            if cyclecounter > pollingRateInHertz * statePublishIntervalInSeconds:
                self.publishMapState()
                cyclecounter = 0

            self.syncLock.release()

            rate.sleep()


        # We only write the debug image on exit as this is a very expensive operation on tiny devices such as a RPI.
        rospy.loginfo('Writing debug image...')
        image = self.map.toDebugImage()
        cv2.imwrite(debugimagelocation, image)

        rospy.loginfo('Pathmanager terminated.')


if __name__ == '__main__':
    try:
        manager = PathManager()
        manager.start()
    except rospy.ROSInterruptException:
        pass
