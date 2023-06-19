from priorityqueue import PriorityQueue

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion
from trigfunctions import normalizerad, distance

import math
import rospy

class VerticalSlice:

    def __init__(self, x, width, ystart, yend):
        self.x = x
        self.width = width
        self.ystart = ystart
        self.yend = yend
        self.leftslices = []
        self.rightslices = []

    def __str__(self):
        return "(VerticalSlice x=" + str(self.x) + ", ystart=" + str(self.ystart) + ", yend=" + str(self.yend) + ", width=" + str(self.width) + ")"

    def __repr__(self):
        return self.__str__()

    def covers(self, position):
        x, y = position
        return x >= self.x and x < self.x + self.width and y >= self.ystart and y < self.yend

    def top(self):
        return int(self.x + self.width / 2), self.ystart

    def bottom(self):
        return int(self.x + self.width / 2), self.yend

class Map:

    def __init__(self):
        self.latestmap = None

    def newmap(self, message):
        self.latestmap = message

    def indexfor(self, x, y):
        if self.latestmap is None:
            return None

        if x < 0 or y < 0 or x > self.latestmap.info.width or y > self.latestmap.info.height:
            return None

        return x + (y * self.latestmap.info.width)

    def posetogrid(self, pose):
        return self.xytogrid(pose.pose.position.x, pose.pose.position.y)

    def xytogrid(self, x, y):
        mapx = int((x - self.latestmap.info.origin.position.x) / self.latestmap.info.resolution)
        mapy = int((y - self.latestmap.info.origin.position.y) / self.latestmap.info.resolution)
        return mapx, mapy

    def gridtopose(self, g):
        x, y = g
        pose = PoseStamped()
        pose.pose.position.x = self.latestmap.info.origin.position.x + x * self.latestmap.info.resolution + self.latestmap.info.resolution / 2
        pose.pose.position.y = self.latestmap.info.origin.position.y + y * self.latestmap.info.resolution + self.latestmap.info.resolution / 2
        return pose

    def posetoyaw(self, pose):
        quat = pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return normalizerad(yaw)

    def isdirectpathpose(self, srcpose, targetpose):

        return self.isdirectpath(self.posetogrid(srcpose), self.posetogrid(targetpose))

    def isdirectpath(self, src, target):

        # Code adapted from https://jccraig.medium.com/we-must-draw-the-line-1820d49d19dd
        (x1, y1) = src
        (x2, y2) = target

        x = x1
        y = y1
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        sx = 1 if x1 < x2 else -1 if x1 > x2 else 0
        sy = 1 if y1 < y2 else -1 if y1 > y2 else 0
        ix = dy // 2
        iy = dx // 2
        pixels = dx + 1 if dx > dy else dy + 1
        while pixels:

            idx = self.indexfor(x, y)
            if idx is None or self.latestmap.data[idx] != 0:
                return False

            ix += dx
            if ix >= dy:
                ix -= dy
                x += sx
            iy += dy
            if iy >= dx:
                iy -= dx
                y += sy
            pixels -= 1

        return True

    def findpath(self, srcpose, targetpose):

        src = self.posetogrid(srcpose)
        target = self.posetogrid(targetpose)

        return self.findpathgrid(src, target)

    def findpathgrid(self, src, target):

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put((id(src), src), 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        scandistance = 5

        def adjecentcellsfor(src):
            cells = []

            if self.isdirectpath(src, target):
                cells.append(target)
                return cells

            (x, y) = src

            offsetstocheck = ((-scandistance, scandistance), (0, scandistance), (scandistance, scandistance), (-scandistance, 0), (scandistance, 0), (-scandistance, -scandistance), (0, -scandistance), (-scandistance, -scandistance))
            for off in offsetstocheck:
                (offx, offy) = off
                cell = (x + offx, y + offy)
                if self.isdirectpath(src, cell):
                    cells.append(cell)

            return cells

        def compress(path):
            result = []
            for i, cell in enumerate(path):
                if i == 0 or i == len(path) - 1:
                    # We have to keep the first and the last point of the path
                    result.append(cell)
                else:
                    # We check the trajectory from the previous point to the current
                    (px, py) = path[i - 1]
                    (cx, cy) = cell
                    (nx, ny) = path[i + 1]

                    tprev = math.atan2(cy - py, cx - px)
                    tnext = math.atan2(ny - cy, nx - cx)

                    if abs(tnext - tprev) > 0.05: # if there is a significant change in angle, add it to the list
                        result.append(cell)

            return result


        while not frontier.empty():
            _, currentcell = frontier.get()

            if currentcell == target:
                completepath = []
                while currentcell != src:
                    completepath.append(currentcell)
                    currentcell = came_from[currentcell]

                completepath.append(src)
                completepath.reverse()

                return compress(completepath)

            for nextCell in adjecentcellsfor(currentcell):
                new_cost = cost_so_far[currentcell] + distance(currentcell, nextCell)
                if nextCell not in cost_so_far or new_cost < cost_so_far[nextCell]:
                    cost_so_far[nextCell] = new_cost
                    priority = new_cost + distance(nextCell, target)
                    frontier.put((id(nextCell), nextCell), priority)
                    came_from[nextCell] = currentcell

        return None

    def celldecompose(self, scanwidth, areatoscan):

        minx = 0
        maxx = self.latestmap.info.width

        miny = 0
        maxy = self.latestmap.info.height

        if areatoscan is not None:

            minx, miny = self.xytogrid(areatoscan.mapTopLeftX, areatoscan.mapBottomRightY)
            maxx, maxy = self.xytogrid(areatoscan.mapBottomRightX, areatoscan.mapTopLeftY)

            rospy.loginfo("Calculating full coverage path from (%s, %s) to (%s, %s)", minx, miny, maxx, maxy)

        x = minx

        slices = []

        def testslice(x, y, testwidth):
            idx = x + (y * self.latestmap.info.width)
            for i in range(testwidth):
                if self.latestmap.data[idx] != 0:
                    return False
                idx = idx + 1

            return True

        def commonborder(left, right):
            if left.ystart <= right.ystart <= left.yend:
                return True
            if left.ystart <= right.yend <= left.yend:
                return True
            if right.ystart < left.ystart and right.yend > left.yend:
                return True

            return False

        prevslices = []

        while x < maxx:
            y = miny
            testwidth = min(scanwidth, maxx - 1 - x)

            if testwidth > 0:

                lateststart = None

                currentslices = []

                while y < maxy:

                    if testslice(x, y, testwidth):
                        if lateststart is None:
                            lateststart = y
                    else:
                        if lateststart is not None:
                            newslice = VerticalSlice(x, testwidth, lateststart, y - 1)

                            slices.append(newslice)
                            currentslices.append(newslice)

                            lateststart = None

                    y = y + 1

                # We have to finish a running slice if there exists one
                if lateststart is not None:
                    newslice = VerticalSlice(x, testwidth, lateststart, y - 1)
                    slices.append(newslice)
                    currentslices.append(newslice)

                for prev in prevslices:
                    for curr in currentslices:
                        if commonborder(prev, curr):
                            prev.rightslices.append(curr)
                            curr.leftslices.append(prev)

                prevslices = currentslices

            x = x + scanwidth

        return slices

    def fullcoveragepath(self, scanwidth, currentpose, cleanarea):

        slices = self.celldecompose(scanwidth, cleanarea)
        robotposition = self.posetogrid(currentpose)

        startslice = None
        for sl in slices:
            if sl.covers(robotposition):
                startslice = sl

        if startslice is None:
            rospy.loginfo('fullcoveragepath(): No starting slice at %s found in %s', robotposition, slices)
            return None, None, None

        slicetop = startslice.top()
        slicebottom = startslice.bottom()

        coveragepath = []

        currentpos = None

        if distance(robotposition, slicetop) < distance(robotposition, slicebottom):
            # Move robot to top and then to the bottom of the slice
            coveragepath.append(slicetop)
            coveragepath.append(slicebottom)
            currentpos = slicebottom
        else:
            # Move robot to the botton and then to the top of the slice
            coveragepath.append(slicebottom)
            coveragepath.append(slicetop)
            currentpos = slicetop

        # Now, we do have all slices
        # Step: find the slice containing the robot, this is the starting slice
        # within the starting slice, check the shortest path to the top or to the bottom, move to the shortest point
        # 1. Move within the slice to the opposite end
        # 2. Mark the current slice as visited
        # 3. Check all not visited neighboring slices of the current sliice. Move to the shortest slice and repeat at 1. Mark all other slices as open
        # 4. If there are no direct neighbors, select the nearest slice from the open list and repeat at 1
        # 5. Repeat until open list is empty

        def dfs(level, currentslice, currentpos, visited, coveragepath):

            if not currentslice in visited:

                visited.append(currentslice)

                def nearestslice(currentpos):

                    nearest = None
                    lastdistance = None

                    slicestocheck = currentslice.leftslices + currentslice.rightslices
                    for sl in slicestocheck:
                        if sl not in visited:
                            dist = min(distance(currentpos, sl.top()), distance(currentpos, sl.bottom()))
                            if nearest is None:
                                nearest = sl
                                lastdistance = dist
                            else:
                                if dist < lastdistance:
                                    nearest = sl
                                    lastdistance = dist

                    return nearest

                nextslice = nearestslice(currentpos)
                while nextslice:

                    top = nextslice.top()
                    bottom = nextslice.bottom()

                    if distance(currentpos, top) < distance(currentpos, bottom):
                        # Move to top, then to the bottom

                        path = self.findpathgrid(currentpos, top)
                        if path:
                            coveragepath.extend(path)
                            coveragepath.append(bottom)
                            currentpos = bottom
                        else:
                            return currentpos, top
                    else:
                        # Move to the bottom, then to the top
                        path = self.findpathgrid(currentpos, bottom)
                        if path:
                            coveragepath.extend(path)
                            coveragepath.append(top)
                            currentpos = top
                        else:
                            return currentpos, bottom

                    currentpos, error = dfs(level + 1, nextslice, currentpos, visited, coveragepath)
                    #if error:
                    #    return currentpos, error

                    nextslice = nearestslice(currentpos)

                return currentpos, None

            return currentpos, None

        currentpos, error = dfs(0, startslice, currentpos, [], coveragepath)

        return coveragepath, currentpos, error

    def gridpointstoposes(self, points, mapframe):
        path = Path()
        path.header.frame_id = mapframe
        path.header.stamp = rospy.Time.now()

        for po in points:
            path.poses.append(self.gridtopose(po))


        return path

