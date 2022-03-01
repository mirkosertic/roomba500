#!/usr/bin/env python3

import rospy
import cv2

from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from tf.transformations import quaternion_from_euler

from map import GridCellStatus, NavigationMap


class ReadMap:

    def __init__(self):
        self.navigationmap = NavigationMap()
        self.markerstopic = None
        self.currentposition = None

    def newMoveBaseSimpleGoalMessage(self, message):

        quat = quaternion_from_euler(0, 0, 0)
        markers = MarkerArray()

        if self.currentposition is None:
            current = self.navigationmap.nearestCellCovering(message.pose.position)
            if current is not None:
                self.currentposition = current

                m1 = Marker()
                m1.header.frame_id = 'map'
                m1.header.stamp = rospy.Time.now()
                m1.id = 0
                m1.type = 1
                m1.action = 0
                m1.scale.x = self.navigationmap.scanwidthinmeters
                m1.scale.y = self.navigationmap.scanwidthinmeters
                m1.scale.z = self.navigationmap.scanwidthinmeters
                m1.pose.position.x = self.currentposition.centerx
                m1.pose.position.y = self.currentposition.centery
                m1.pose.position.z = 0.0
                m1.pose.orientation.x = quat[0]
                m1.pose.orientation.y = quat[1]
                m1.pose.orientation.z = quat[2]
                m1.pose.orientation.w = quat[3]
                m1.points = []
                m1.colors = []
                m1.color.a = 0.3
                m1.color.r = 0
                m1.color.g = 0
                m1.color.b = 1

                markers.markers.append(m1)
            else:
                rospy.loginfo("Cannot find initial cell near to %s:%s", message.pose.position.x, message.pose.position.y)
        else:
            target = self.navigationmap.nearestCellCovering(message.pose.position)
            if target is not None:
                #path = self.navigationmap.findPath(self.currentposition, target)
                path = self.navigationmap.startCoverageBoustrophedon(target)

                m1 = Marker()
                m1.header.frame_id = 'map'
                m1.header.stamp = rospy.Time.now()
                m1.id = 0
                m1.type = 1
                m1.action = 0
                m1.scale.x = self.navigationmap.scanwidthinmeters
                m1.scale.y = self.navigationmap.scanwidthinmeters
                m1.scale.z = self.navigationmap.scanwidthinmeters
                m1.pose.position.x = self.currentposition.centerx
                m1.pose.position.y = self.currentposition.centery
                m1.pose.position.z = 0.0
                m1.pose.orientation.x = quat[0]
                m1.pose.orientation.y = quat[1]
                m1.pose.orientation.z = quat[2]
                m1.pose.orientation.w = quat[3]
                m1.points = []
                m1.colors = []
                m1.color.a = 0.3
                m1.color.r = 0
                m1.color.g = 0
                m1.color.b = 1
                markers.markers.append(m1)

                m2 = Marker()
                m2.header.frame_id = 'map'
                m2.header.stamp = rospy.Time.now()
                m2.id = 1
                m2.type = 1
                m2.action = 0
                m2.scale.x = self.navigationmap.scanwidthinmeters
                m2.scale.y = self.navigationmap.scanwidthinmeters
                m2.scale.z = self.navigationmap.scanwidthinmeters
                m2.pose.position.x = target.centerx
                m2.pose.position.y = target.centery
                m2.pose.position.z = 0.0
                m2.pose.orientation.x = quat[0]
                m2.pose.orientation.y = quat[1]
                m2.pose.orientation.z = quat[2]
                m2.pose.orientation.w = quat[3]
                m2.points = []
                m2.colors = []
                m2.color.a = 0.3
                m2.color.r = 1
                m2.color.g = 0
                m2.color.b = 1
                markers.markers.append(m2)

                if path is not None:

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

                    self.currentposition = target
                else:
                    rospy.loginfo("Cannot find path from %s:%s to %s:%s", self.currentposition.centerx, self.currentposition.centery, target.centerx, target.centery)
            else:
                rospy.loginfo("Cannot find target cell near to %s:%s", message.pose.position.x, message.pose.position.y)

        if len(markers.markers) > 0:
            self.markerstopic.publish(markers)

        pass

    def start(self):
        rospy.init_node('test', anonymous=True)

        rospy.loginfo('Waiting for service')
        rospy.wait_for_service('static_map')
        rospy.loginfo('Invoking service')
        service = rospy.ServiceProxy('static_map', GetMap)
        response = service()

        map = response.map

        self.navigationmap.initOrUpdateFromOccupancyGrid(map)

        markers = MarkerArray()

        for cell in self.navigationmap.cells:

            if cell.status == GridCellStatus.OCCUPIED or cell.status == GridCellStatus.FREE:
                quat = quaternion_from_euler(0, 0, 0)
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = rospy.Time.now()
                marker.id = int(cell.centerx * 100000000 + cell.centery * 100)
                marker.type = 1
                marker.action = 0
                marker.scale.x = self.navigationmap.scanwidthinmeters
                marker.scale.y = self.navigationmap.scanwidthinmeters
                marker.scale.z = self.navigationmap.scanwidthinmeters
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

        image = self.navigationmap.toDebugImage()
        cv2.imwrite('../maps/loaded.png', image)

        mappub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        mappub.publish(map)

        self.markerstopic = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10, latch=True)
        self.markerstopic.publish(markers)

        rospy.loginfo('Generated %s markers', len(markers.markers))

        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.newMoveBaseSimpleGoalMessage)

        # Processing the sensor polling in an endless loop until this node shuts down
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        m = ReadMap()
        m.start()
    except rospy.ROSInterruptException:
        pass
