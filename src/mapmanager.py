#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from priorityqueue import PriorityQueue


class MapManager:

    def __init__(self, markertopic, debugimagelocation):
        self.latestMap = None
        self.robotNavigationGrid = {}
        self.robotDiameterInMeters = 0.30
        self.occupancyThreshold = .50
        self.markertopic = markertopic
        self.debugImageLocation = debugimagelocation

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

    def compress(self, path):
        # TODO: the first navigation point with a changing direction is the target point, not just the second one
        result = []
        for i, point in enumerate(path):
            if i == 0 or i == len(path) - 1:
                # We have to keep the first and the last point of the path
                result.append(point)
            else:
                # We check the trajectory from the previous point to the current
                (xp, yp) = path[i - 1]
                (xc, yc) = point
                (xn, yn) = path[i + 1]

                tprev = math.atan2(yc - yp, xc - xp)
                tnext = math.atan2(yn - yc, xn - xc)

                if abs(tnext - tprev) > math.pi / 8:  # Change greater than 22.5 degrees
                    result.append(point)

        return result

    def findPath(self, src, target):

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put(src, 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        def clampDegrees(value):
            while (value < 0):
                value += 360
            while (value >= 360):
                value -= 360
            return value

        def toDegrees(value):
            return clampDegrees(math.degrees(value))

        def heuristicscore(src, target):
            x1, y1 = src
            x2, y2 = target
            dx = x2 - x1
            dy = y2 - y1
            #return math.sqrt(dx * dx + dy * dy)
            return abs(dx) + abs(dy)

        def adjecentcellsfor(src):
            x, y = src
            neighbours = []
            for cell, status in self.robotNavigationGrid.items():
                if status is False and cell != src:
                    cellx, celly = cell
                    dx = cellx - x
                    dy = celly - y
                    distance = math.sqrt(dx * dx + dy * dy)
                    if distance < self.robotDiameterInMeters * 1.5:
                        neighbours.append(cell)

            left = None
            topleft = None
            top = None
            topright = None
            right = None
            bottomleft = None
            bottom = None
            bottomright = None

            for cell in neighbours:
                cellx, celly = cell
                angle = toDegrees(math.atan2(celly - y, cellx - x))
                print(angle)
                if angle < 10 or angle > 350:
                    right = cell
                if angle > 35 and angle < 55:
                    topright = cell
                if angle > 80 and angle < 100:
                    top = cell
                if angle > 125 and angle < 145:
                    topleft = cell
                if angle > 170 and angle < 190:
                    left = cell
                if angle > 215 and angle < 235:
                    bottomleft = cell
                if angle > 260 and angle < 280:
                    bottom = cell
                if angle > 305 and angle < 325:
                    bottomright = cell

            cells = []
            if left is not None:
                cells.append(left)
            if right is not None:
                cells.append(right)
            if top is not None:
                cells.append(top)
            if bottom is not None:
                cells.append(bottom)

            if top is not None:
                if right is not None and topright is not None:
                    cells.append(topright)
                if left is not None and topleft is not None:
                    cells.append(topleft)
            if bottom is not None:
                if right is not None and bottomright is not None:
                    cells.append(bottomright)
                if left is not None and bottomleft is not None:
                    cells.append(bottomleft)

            return cells

        while not frontier.empty():
            currentcell = frontier.get()

            if currentcell == target:
                completePath = []
                while currentcell != src:
                    completePath.append(currentcell)
                    currentcell = came_from[currentcell]

                completePath.reverse()

                return self.compress(completePath)

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
            for x2 in range(x, x + math.floor(scansquaresize)):
                for y2 in range(y, y + math.floor(scansquaresize)):
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

        if self.debugImageLocation is not None:

            cv2.imwrite(self.debugImageLocation, image)
