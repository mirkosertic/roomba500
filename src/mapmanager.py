#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from priorityqueue import PriorityQueue
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class MapManager:

    def __init__(self, markertopic):
        self.latestMap = None
        self.robotNavigationGrid = {}
        self.robotDiameterInMeters = 0.30
        self.occupancyThreshold = .50
        self.markertopic = markertopic

    def create_blank(self, w, h, rgb_color=(0, 0, 0)):

        """Create new image(numpy array) filled with certain color in RGB"""
        # Create black blank image
        image = np.zeros((h, w, 3), np.uint8)

        # Since OpenCV uses BGR, convert the color first
        color = tuple(reversed(rgb_color))
        # Fill image with color
        image[:] = color

        return image

    def nearestNavigationPointTo(self, position):
        lastdistance = None
        foundposition = None
        for (centerX, centerY), status in self.robotNavigationGrid.items():
            deltax = position.x - centerX
            deltay = position.y - centerY
            distance = math.sqrt(deltax * deltax + deltay * deltay)
            if lastdistance is None or lastdistance > distance:
                lastdistance = distance
                foundposition = (centerX, centerY)

        return foundposition

    def highlightPath(self, points):

        quat = quaternion_from_euler(0, 0, 0)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
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

        for (centerx, centery) in points:
            marker.points.append(Point(centerx, centery, 0.05))
            marker.colors.append(ColorRGBA(0, 0, 1, 1))

        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1

        markers = MarkerArray()
        markers.markers.append(marker)
        self.markertopic.publish(markers)


    def findPath(self, src, target):

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put(src, 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        def heuristicscore(src, target):
            x1, y1 = src
            x2, y2 = target
            dx = x2 - x1
            dy = y2 - y1
            #return math.sqrt(dx * dx + dy * dy)
            return abs(dx) + abs(dy)

        def adjecentcellsfor(src):
            x, y = src
            freeadjecentcells = []
            for cell, status in self.robotNavigationGrid.items():
                if status is False and cell != src:
                    cellx, celly = cell
                    dx = cellx - x
                    dy = celly - y
                    distance = math.sqrt(dx * dx + dy * dy)
                    if distance < self.robotDiameterInMeters * 1.5:
                        freeadjecentcells.append(cell)

            return freeadjecentcells

        while not frontier.empty():
            currentcell = frontier.get()

            if currentcell == target:
                completePath = []
                while currentcell != src:
                    completePath.append(currentcell)
                    currentcell = came_from[currentcell]

                completePath.reverse()
                return completePath

            for nextCell in adjecentcellsfor(currentcell):
                new_cost = cost_so_far[currentcell] + heuristicscore(currentcell, nextCell)
                if nextCell not in cost_so_far or new_cost < cost_so_far[nextCell]:
                    cost_so_far[nextCell] = new_cost
                    priority = new_cost + heuristicscore(nextCell, target)
                    frontier.put(nextCell, priority)
                    came_from[nextCell] = currentcell

        return None

    def newMapData(self, message):

        mapwidth = message.info.width
        mapheight = message.info.height
        mapresolution = message.info.resolution

        originpose = message.info.origin

        orientationquat = originpose.orientation
        (_, _, mapyaw) = euler_from_quaternion([orientationquat.x, orientationquat.y, orientationquat.z, orientationquat.w])

        rospy.loginfo("Received map with resolution = %s, width = %s, height = %s, origin = %s, rotation = %s ", mapresolution, mapwidth, mapheight, originpose.position, mapyaw)

        scansquaresize = self.robotDiameterInMeters / mapresolution

        def isOccupied(x, y):
            containsunknownareas = False
            for x2 in range(x, x + math.ceil(scansquaresize)):
                for y2 in range(y, y + math.ceil(scansquaresize)):
                    probability = message.data[(mapwidth - y2 - 1) * mapwidth + x2]
                    if probability == -1:
                        containsunknownareas = True

                    if probability >= self.occupancyThreshold:
                        return True

            if containsunknownareas:
                return None

            return False

        # We do a marching square scan to split the occupancy grid
        # into grid cells with the same size as the robot. These
        # cells form the basis for the navigation map
        for x in range(0, mapwidth - int(scansquaresize) - 1, int(scansquaresize)):
            for y in range(0, mapheight - int(scansquaresize) - 1, int(scansquaresize)):
                status = isOccupied(x, y)
                if status is True or status is False:
                    # TODO: respect orientation and grid snapping!
                    mapx = originpose.position.x + (x + scansquaresize / 2) * mapresolution
                    mapy = originpose.position.y + (mapheight - 1 - y - scansquaresize / 2) * mapresolution
                    self.robotNavigationGrid[(mapx, mapy)] = status

        self.latestMap = message

        self.writeDebugOutput()

    def writeDebugOutput(self):

        if self.latestMap is None:
            return

        mapwidth = self.latestMap.info.width
        mapheight = self.latestMap.info.height
        mapresolution = self.latestMap.info.resolution
        originpose = self.latestMap.info.origin

        scansquaresize = int(self.robotDiameterInMeters / mapresolution)

        image = self.create_blank(mapwidth, mapheight, rgb_color=(255, 255, 255))

        # Write the map
        for y in range(mapheight):
            for x in range(mapwidth):
                color = None
                probability = self.latestMap.data[(mapwidth - y - 1) * mapwidth + x]
                if probability == -1:
                    # Unknown
                    color = (200, 200, 200)
                else:
                    if probability >= self.occupancyThreshold:
                        # occupied
                        color = (0, 0, 0)
                    else:
                        # free
                        color = (255, 255, 255)

                image[y, x] = color

        markers = MarkerArray()

        # Now we write the occupancy grid
        for (centerX, centerY), status in self.robotNavigationGrid.items():
            color = (0, 255, 0)
            if status is True:
                color = (0, 0, 255)

            centerxingrid = int((centerX - originpose.position.x) / mapresolution)
            centeryingrid = mapheight - 1 - int((centerY - originpose.position.y) / mapresolution)

            # Draw a square centered at the point in the grid
            topx = centerxingrid - int(scansquaresize / 2)
            topy = centeryingrid - int(scansquaresize / 2)

            for a in range(scansquaresize):
                image[topy, topx + a] = color  # Top line
                image[topy + scansquaresize - 1, topx + a] = color  # Bottom line
                image[topy + a, topx] = color  # Left line
                image[topy + a, topx + scansquaresize - 1] = color  # Right line

            g, b, r = color

            quat = quaternion_from_euler(0, 0, 0)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.id = int(centerX * 100000000 + centerY * 100)
            marker.type = 1
            marker.action = 0
            marker.scale.x = self.robotDiameterInMeters
            marker.scale.y = self.robotDiameterInMeters
            marker.scale.z = self.robotDiameterInMeters
            marker.pose.position.x = centerX
            marker.pose.position.y = centerY
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            marker.color.a = 0.25
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            markers.markers.append(marker)


        self.markertopic.publish(markers)
        cv2.imwrite('/mnt/d/Mirko/ownCloud/Schaltungen/catkin_ws/src/roomba500/maps/running.png', image)
