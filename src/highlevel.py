#!/usr/bin/env python3

import rospy
import threading
import tf
import os
import pathlib
import yaml
import math
import traceback

from map import Map
from robotcontroller import RobotController
from driver import Driver
from robotbehaviors import RotateToState, DriveToPosition

from std_msgs.msg import Int16, ColorRGBA
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, Quaternion, Vector3, PoseWithCovarianceStamped, Polygon, Point32

from roomba500.msg import NavigationInfo
from roomba500.srv import Clean, CleanResponse, Cancel, CancelResponse, UpdateNoCleanZones, UpdateNoCleanZonesResponse, MoveTo, MoveToResponse

from costmap_prohibition_layer.srv import UpdateZones, UpdateZonesResponse

class Highlevel:

    def __init__(self):
        self.syncLock = threading.Lock()

        self.navmap = None
        self.driver = None
        self.robotcontroller = None

        self.transformlistener = None
        self.infotopic = None

        self.cleaningpathtopic = None

        self.sidebrushpub = None
        self.mainbrushpub = None
        self.vacuumpub = None

        self.mapareas = []
        self.mapareasfile = None

        self.navigationpathtopic = None

    def newOdomMessage(self, data):

        rospy.logdebug("Received new odometry data")

        self.syncLock.acquire()

        try:
            self.robotcontroller.newodometry(data)
        except Exception as e:
            rospy.loginfo('Error while processing odometry data : %s', traceback.format_exc())

        self.syncLock.release()

    def newShutdownCommand(self, req):

        rospy.loginfo("Received shutdown command")

        self.syncLock.acquire()

        self.stopVacuum()

        rospy.signal_shutdown('Shutdown requested')

        self.syncLock.release()

    def clean(self, req):
        rospy.loginfo("Received clean command")

        self.syncLock.acquire()

        try:

            path, lastpos, error = self.navmap.fullcoveragepath(3, self.robotcontroller.latestodominmapframe, req.cleanArea)

            if path is not None:
                rospy.loginfo("Cleaning coverage path computed")

                cp = self.navmap.gridpointstoposes(path, 'map')
                self.cleaningpathtopic.publish(cp)

                self.pathtorobotstatemachine(path)
                rospy.loginfo('Navigation state chain computed')

            else:
                rospy.loginfo("Cleaning coverage path not computed")

        except Exception as e:
            rospy.loginfo('Error in clean command : %s', traceback.format_exc())

        self.syncLock.release()

        return CleanResponse(0)

    def cancel(self, req):
        rospy.loginfo("Received cancel command")

        self.syncLock.acquire()

        self.stopVacuum()
        self.robotcontroller.stop()

        self.syncLock.release()

        return CancelResponse(0)

    def pathtorobotstatemachine(self, path):

        x = len(path) - 1
        while x >= 1:
            (xtarget, ytarget) = path[x]
            (xorigin, yorigin) = path[x - 1]

            pose = self.navmap.gridtopose(path[x])

            self.robotcontroller.appendbehavior(DriveToPosition(pose))

            dx = xtarget - xorigin
            dy = ytarget - yorigin

            yaw = math.atan2(dy, dx)
            self.robotcontroller.appendbehavior(RotateToState(yaw))

            x = x - 1

        # Calculate to first waypoint
        (xtarget, ytarget) = path[1]
        (xorigin, yorigin) = path[0]

        pose = self.navmap.gridtopose(path[1])

        self.robotcontroller.appendbehavior(DriveToPosition(pose))

        dx = xtarget - xorigin
        dy = ytarget - yorigin

        yaw = math.atan2(dy, dx)
        self.robotcontroller.appendbehavior(RotateToState(yaw))

    def moveto(self, req):
        rospy.loginfo("Received moveto command")

        self.syncLock.acquire()

        target = req.goal

        try:
            navgoalinframe = self.transformlistener.transformPose('map', target)
            rospy.loginfo('Planning path...')

            p = self.navmap.findpath(self.robotcontroller.latestodominmapframe, navgoalinframe)
            if p is not None:

                path = self.navmap.gridpointstoposes(p, 'map')

                rospy.loginfo('Path calculated with %s poses', len(path.poses))
                self.navigationpathtopic.publish(path)

                self.pathtorobotstatemachine(p)

                rospy.loginfo('Navigation state chain computed')

            else:
                rospy.loginfo('No path found')

        except Exception as e:
            rospy.loginfo('Error creating path : %s', traceback.format_exc())

        self.syncLock.release()

        return MoveToResponse(0)

    def updateNoCleanZones(self, req):
        rospy.loginfo("Received UpdateNoCleanZones command")

        self.syncLock.acquire()

        try:
            for area in req.areas:
                self.mapareas.append(dict(
                    type = "prohibited",
                    mapTopLeftX = area.mapTopLeftX,
                    mapTopLeftY = area.mapTopLeftY,
                    mapBottomRightX = area.mapBottomRightX,
                    mapBottomRightY = area.mapBottomRightY
                ))

            rospy.loginfo('Saving map area data to %s', self.mapareasfile)
            with open(self.mapareasfile, "w") as outfile:
                data = dict(
                    mapareas = self.mapareas
                )
                yaml.dump(data, outfile, default_flow_style=False)

            self.updateProhibitedAreasInCostMap()

        except Exception as e:
            rospy.loginfo('Error updating no clean zones : %s', traceback.format_exc())

        self.syncLock.release()

        return UpdateNoCleanZonesResponse(0)

    def updateProhibitedAreasInCostMap(self):

        zones = []
        for area in self.mapareas:
            if area["type"] == "prohibited":
                p = Polygon()
                p.points.append(Point32(area["mapTopLeftX"], area["mapTopLeftY"], .0))
                p.points.append(Point32(area["mapBottomRightX"], area["mapTopLeftY"], .0))
                p.points.append(Point32(area["mapBottomRightX"], area["mapBottomRightY"], .0))
                p.points.append(Point32(area["mapTopLeftX"], area["mapBottomRightY"], .0))
                zones.append(p)

        if len(zones) > 0:

            servicename = "costmap_node/costmap/prohibited_layer/update_zones"

            rospy.loginfo("Calling service %s", servicename)
            rospy.wait_for_service(servicename)
            service = rospy.ServiceProxy(servicename, UpdateZones)
            res = service(zones)

            rospy.loginfo("Zones updated")

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
        self.vacuumpub.publish(Int16(127))

    def stopVacuum(self):
        self.mainbrushpub.publish(Int16(0))
        self.sidebrushpub.publish(Int16(0))
        self.vacuumpub.publish(Int16(0))

    def start(self):
        rospy.init_node('highlevel', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '5'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.transformlistener = tf.TransformListener()

        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))
        self.navmap = Map()
        self.robotcontroller = RobotController(self.transformlistener, 'map', self.driver)

        # We use the provided costmap from ROS navigation stack by default for further path finding
        rospy.Subscriber(rospy.get_param('~costmaptopic', 'costmap_node/costmap/costmap'), OccupancyGrid, self.navmap.newmap)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        self.infotopic = rospy.Publisher('navigation_info', NavigationInfo, queue_size=10)
        self.cleaningpathtopic = rospy.Publisher('cleaningpath', Path, queue_size=10)
        self.navigationpathtopic = rospy.Publisher('navpath', Path, queue_size=10)

        # Motor control
        self.mainbrushpub = rospy.Publisher('roomba/cmd_mainbrush', Int16, queue_size=10)
        self.sidebrushpub = rospy.Publisher('roomba/cmd_sidebrush', Int16, queue_size=10)
        self.vacuumpub = rospy.Publisher('roomba/cmd_vacuum', Int16, queue_size=10)

        # We consume odometry here
        rospy.Subscriber("odom", Odometry, self.newOdomMessage)

        # Register services
        rospy.Service("clean", Clean, self.clean)
        rospy.Service("cancel", Cancel, self.cancel)
        rospy.Service("updatenocleanzones", UpdateNoCleanZones, self.updateNoCleanZones)
        rospy.Service("moveto", MoveTo, self.moveto)

        self.mapareasfile = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('mapareas.yaml'))
        if os.path.exists(self.mapareasfile):
            rospy.loginfo("Reading map area data from %s", self.mapareasfile)
            with open(self.mapareasfile, 'r') as stream:
                data = yaml.safe_load(stream)
                self.mapareas = data["mapareas"]
                self.updateProhibitedAreasInCostMap()

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Highlevel Controller is ready.")

        while not rospy.is_shutdown():
            # We do nothing really here, except publishing the current navigation status at a very low frequency
            # TODO
            rate.sleep()

        rospy.loginfo('Highlevel Controller terminated.')


if __name__ == '__main__':
    try:
        node = Highlevel()
        node.start()
    except rospy.ROSInterruptException:
        pass
