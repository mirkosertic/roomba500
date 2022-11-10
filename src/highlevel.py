#!/usr/bin/env python3

import rospy
import threading
import tf
import actionlib
import math

from map import NavigationMap, GridCellStatus

from std_msgs.msg import Int16, ColorRGBA
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Vector3, PoseWithCovarianceStamped, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from roomba500.msg import NavigationInfo
from roomba500.srv import Clean, CleanResponse, Cancel, CancelResponse, UpdateNoCleanZones, UpdateNoCleanZonesResponse

from costmap_prohibition_layer.srv import UpdateZones, UpdateZonesResponse

class Highlevel:

    def __init__(self):
        self.syncLock = threading.Lock()

        self.latestOdometryPose = None
        self.latestOdometryFrame = None

        self.map = None
        self.transformlistener = None
        self.infotopic = None
        self.cleaningmaptopic = None
        self.movebaseclient = None
        self.currentpathindex = 0
        self.cleaningpathtopic = None

        self.sidebrushpub = None
        self.mainbrushpub = None
        self.vacuumpub = None

    def newOdomMessage(self, data):

        rospy.logdebug("Received new odometry data")

        self.syncLock.acquire()

        self.latestOdometryPose = data.pose.pose
        self.latestOdometryFrame = data.header.frame_id

        self.syncLock.release()

    def newInitialPose(self,data):

        rospy.loginfo("Received new initial pose")

        self.syncLock.acquire()

        self.latestOdometryPose = data.pose.pose
        self.latestOdometryFrame = data.header.frame_id

        self.syncLock.release()

    def shortestAngle(self, origin, target):
        # Code taken from https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles/28037434
        diff = (target - origin + 180) % 360 - 180
        if diff < -180:
            return diff + 360
        return diff

    def clampDegrees(self, value):
        while value < 0:
            value += 360
        while value >= 360:
            value -= 360

        return value

    def toDegrees(self, value):
        return self.clampDegrees(math.degrees(value))

    def poseAngleInDegrees(self, pose):
        quat = pose.orientation
        (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return self.toDegrees(yaw)

    def followPath(self, path, frameid):

        rospy.loginfo("Path to target has : %s waypoints", str(len(path)))

        self.currentpathindex = 0

        def poseAtIndex(index):
            # We calculate the target rotation for the robot. It should face to the next waypoint
            # of the path
            if index < len(path) - 1:
                targetangle = math.atan2(path[index + 1].centery - path[index].centery, path[index + 1].centerx - path[index].centerx)
                q = quaternion_from_euler(0, 0, targetangle)
            else:
                # Last node of the path
                # We compute the angle backwards and add 180 degrees to it
                targetangle = math.atan2(path[index - 1].centery - path[index].centery, path[index - 1].centerx - path[index].centerx) + math.pi
                q = quaternion_from_euler(0, 0, targetangle)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = path[index].centerx
            pose.pose.position.y = path[index].centery
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            return pose

        def goalAtIndex(index):
            goal = MoveBaseGoal()
            goal.target_pose = poseAtIndex(index)

            return goal

        def active_cb():
            rospy.loginfo("Goal is now being processing by action server...")
            pass

        def feedback_cb(feedback):
            rospy.logdebug("Got feedback from action server : (%s) %s", str(type(feedback)), str(feedback))

            pose = feedback.base_position
            goal = goalAtIndex(self.currentpathindex)

            dx = goal.target_pose.pose.position.x - pose.pose.position.x
            dy = goal.target_pose.pose.position.y - pose.pose.position.y
            distanceToTarget = math.sqrt(dx * dx + dy * dy)

            shortestAngle = self.shortestAngle(self.poseAngleInDegrees(feedback.base_position.pose), self.poseAngleInDegrees(goal.target_pose.pose))

            self.publishNavigationInfo(distanceToTarget, shortestAngle, self.currentpathindex, len(path))
            pass

        def done_cb(status, result):
            rospy.loginfo("Goal %s marked as done with status %s", str(self.currentpathindex), str(status))

            if status == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal pose %s received a cancel request after it started executing, completed execution!", str(self.currentpathindex + 1))

            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal pose %s reached", str(self.currentpathindex + 1))

                self.currentpathindex = self.currentpathindex + 1

                if self.currentpathindex < len(path):
                    rospy.loginfo("Continue with waypoint %s", str(self.currentpathindex + 1))

                    if not self.movebaseclient.wait_for_server(rospy.Duration(1.0)):
                        rospy.logerr("Action server not available!")
                        rospy.signal_shutdown("Action server not available!")
                        return

                    self.startVacuum()
                    self.movebaseclient.send_goal(goalAtIndex(self.currentpathindex), done_cb, active_cb, feedback_cb)

                    cleaningpath = Path()
                    cleaningpath.header.frame_id = 'map'
                    cleaningpath.header.stamp = rospy.Time.now()
                    for i in range(self.currentpathindex, len(path)):
                        cleaningpath.poses.append(poseAtIndex(i))

                    self.cleaningpathtopic.publish(cleaningpath)
                    return
                else:
                    rospy.loginfo("Path is completed!")
                    self.stopVacuum()

            if status == GoalStatus.ABORTED:
                rospy.loginfo("Goal pose %s was aborted by the action server!", str(self.currentpathindex + 1))
                self.stopVacuum()

            if status == GoalStatus.REJECTED:
                rospy.loginfo("Goal pose %s has been rejected by the action server!", str(self.currentpathindex + 1))
                self.stopVacuum()

            if status == GoalStatus.RECALLED:
                rospy.loginfo("Goal pose %s received a cancel request before it started executing, successful cancellation!", str(self.currentpathindex + 1))
                self.stopVacuum()

            self.publishNavigationInfo(.0, .0, self.currentpathindex, len(path))

            pass

        # We start by navigating to the starting pose
        self.startVacuum()
        self.movebaseclient.send_goal(goalAtIndex(self.currentpathindex), done_cb, active_cb, feedback_cb)

        # Info what needs to be done
        cleaningpath = Path()
        cleaningpath.header.frame_id = 'map'
        cleaningpath.header.stamp = rospy.Time.now()
        for i in range(len(path)):
            cleaningpath.poses.append(poseAtIndex(i))

        self.cleaningpathtopic.publish(cleaningpath)

    def newShutdownCommand(self, req):

        rospy.loginfo("Received shutdown command")

        self.syncLock.acquire()

        self.stopVacuum()

        rospy.signal_shutdown('Shutdown requested')

        self.syncLock.release()

    def clean(self, req):
        rospy.loginfo("Received clean command")

        self.syncLock.acquire()

        odometryInTargetposeFrame = self.latestOdometryTransformedToFrame('map')
        currentposition = self.map.nearestCellCovering(odometryInTargetposeFrame.pose.position)
        if currentposition is not None:
            rospy.loginfo("Calculating full coverage path...")
            path = self.map.startCoverageBoustrophedon(currentposition, req.cleanArea)
            rospy.loginfo("Calculation finished")
            if path is not None:
                self.followPath(path, 'map')

        self.syncLock.release()

        return CleanResponse(0)

    def cancel(self, req):
        rospy.loginfo("Received cancel command")

        self.syncLock.acquire()

        self.movebaseclient.cancel_all_goals()
        self.stopVacuum()

        self.syncLock.release()

        return CancelResponse(0)

    def updateNoCleanZones(self, req):
        rospy.loginfo("Received UpdateNoCleanZones command")

        self.syncLock.acquire()

        zones = []
        for area in req.areas:
            p = Polygon()
            p.points.append(Point32(area.mapTopLeftX, area.mapTopLeftY, .0))
            p.points.append(Point32(area.mapBottomRightX, area.mapTopLeftY, .0))
            p.points.append(Point32(area.mapBottomRightX, area.mapBottomRightY, .0))
            p.points.append(Point32(area.mapTopLeftX, area.mapBottomRightY, .0))
            zones.append(p)

        servicestocall = ["/move_base/global_costmap/prohibited_layer/update_zones", "/move_base/local_costmap/prohibited_layer/update_zones"]
        for servicename in servicestocall:
            rospy.loginfo("Calling service %s", servicename)
            rospy.wait_for_service(servicename)
            service = rospy.ServiceProxy(servicename, UpdateZones)
            res = service(zones)

        rospy.loginfo("Zones updated")

        self.syncLock.release()

        return UpdateNoCleanZonesResponse(0)

    def latestOdometryTransformedToFrame(self, targetframe):

        latestcommontime = self.transformlistener.getLatestCommonTime(targetframe, self.latestOdometryFrame)

        mpose = PoseStamped()
        mpose.pose.position = self.latestOdometryPose.position
        mpose.pose.orientation = self.latestOdometryPose.orientation
        mpose.header.frame_id = self.latestOdometryFrame
        mpose.header.stamp = latestcommontime

        return self.transformlistener.transformPose(targetframe, mpose)

    def publishNavigationInfo(self, distanceToTargetInMeters, angleToTargetInDegrees, currentWaypoint, numWaypoints):
        message = NavigationInfo()
        message.distanceToTargetInMeters = distanceToTargetInMeters
        message.angleToTargetInDegrees = angleToTargetInDegrees
        message.currentWaypoint = currentWaypoint
        message.numWaypoints = numWaypoints
        self.infotopic.publish(message)

    def startVacuum(self):
        self.mainbrushpub.publish(Int16(127))
        self.sidebrushpub.publish(Int16(127))
        self.vacuum.publish(Int16(127))

    def stopVacuum(self):
        self.mainbrushpub.publish(Int16(0))
        self.sidebrushpub.publish(Int16(0))
        self.vacuum.publish(Int16(0))

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

        self.cleaningmaptopic.publish(markers)

    def start(self):
        rospy.init_node('highlevel', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '1'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.cleaningmaptopic = rospy.Publisher('cleaningmap', MarkerArray, queue_size=10, latch=True)

        # This is our map responsible for navigation and pathfinding
        gridcellwidthinmeters = float(rospy.get_param('~gridcellwidthinmeters', '0.18'))
        scanwidthinmeters = float(rospy.get_param('~scanwidthinmeters', '0.18'))

        occupancythreshold = float(rospy.get_param('~occupancythreshold', ' 0'))
        self.map = NavigationMap(gridcellwidthinmeters, scanwidthinmeters, occupancythreshold)

        # We use the provided costmap from ROS navigation stack by default for further path finding
        rospy.Subscriber(rospy.get_param('~costmaptopic', 'move_base/global_costmap/costmap'), OccupancyGrid, self.map.initOrUpdateFromOccupancyGrid)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        self.transformlistener = tf.TransformListener()

        self.infotopic = rospy.Publisher('navigation_info', NavigationInfo, queue_size=10)
        self.cleaningpathtopic = rospy.Publisher('cleaningpath', Path, queue_size=10)

        # Motor control
        self.mainbrushpub = rospy.Publisher('roomba/cmd_mainbrush', Int16, queue_size=10)
        self.sidebrushpub = rospy.Publisher('roomba/cmd_sidebrush', Int16, queue_size=10)
        self.vacuum = rospy.Publisher('roomba/cmd_vacuum', Int16, queue_size=10)

        # We consume odometry here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.newInitialPose)

        # We need the client for move base
        rospy.loginfo("Connecting to move_base action server")
        self.movebaseclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Register services
        rospy.Service("clean", Clean, self.clean)
        rospy.Service("cancel", Cancel, self.cancel)
        rospy.Service("updatenocleanzones", UpdateNoCleanZones, self.updateNoCleanZones)

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Highlevel Controller is ready.")

        while not rospy.is_shutdown():

            self.syncLock.acquire()

            self.publishMapState()

            self.syncLock.release()

            rate.sleep()

        rospy.loginfo('Highlevel Controller terminated.')


if __name__ == '__main__':
    try:
        node = Highlevel()
        node.start()
    except rospy.ROSInterruptException:
        pass
