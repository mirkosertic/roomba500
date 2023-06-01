from priorityqueue import PriorityQueue

from tf.transformations import euler_from_quaternion
from trigfunctions import normalizerad

import math

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
        mapx = int((pose.pose.position.x - self.latestmap.info.origin.position.x) / self.latestmap.info.resolution)
        mapy = int((pose.pose.position.y - self.latestmap.info.origin.position.y) / self.latestmap.info.resolution)
        return (mapx, mapy)

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

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put((id(src), src), 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        scandistance = 5

        def heuristicscore(src, target):
            (x1, y1) = src
            (x2, y2) = target
            dx = x2 - x1
            dy = y2 - y1

            return math.sqrt(dx * dx + dy * dy)

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
                completePath = []
                while currentcell != src:
                    completePath.append(currentcell)
                    currentcell = came_from[currentcell]

                completePath.append(src)
                completePath.reverse()

                return compress(completePath)

            for nextCell in adjecentcellsfor(currentcell):
                new_cost = cost_so_far[currentcell] + heuristicscore(currentcell, nextCell)
                if nextCell not in cost_so_far or new_cost < cost_so_far[nextCell]:
                    cost_so_far[nextCell] = new_cost
                    priority = new_cost + heuristicscore(nextCell, target)
                    frontier.put((id(nextCell), nextCell), priority)
                    came_from[nextCell] = currentcell

        return None
