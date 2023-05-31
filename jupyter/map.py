from priorityqueue import PriorityQueue

from tf.transformations import euler_from_quaternion
from trigfunctions import signedangle
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

    def findpath(self, srcpose, targetpose):

        src = self.posetogrid(srcpose)
        target = self.posetogrid(targetpose)
        srcyaw = self.posetoyaw(srcpose)

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put((id(src), src), 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = (0, srcyaw)

        def distance(src, target):
            (x1, y1) = src
            (x2, y2) = target
            dx = x2 - x1
            dy = y2 - y1

            return math.sqrt(dx * dx + dy * dy)

        def coststoreach(currentyaw, src, target):

            angle = anglebetween(src, target)
            shortestrotation = signedangle(currentyaw, angle)

            d = distance(src, target)

            # We assume 0.3m / second velocity and 90 degrees / second rotation speed
            return d# * 0.3 + abs(shortestrotation) / 15

        def adjecentcellsfor(src):
            cells = []

            def check(x, y, offx, offy):
                idx = self.indexfor(x + offx, y + offy)
                if idx is not None:
                    if self.latestmap.data[idx] == 0:
                        return (x + offx, y + offy)

                return None

            (x, y) = src

            offsetstocheck = ((-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (-1, -1))
            for off in offsetstocheck:
                (offx, offy) = off
                cell = check(x, y, offx, offy)
                if cell is not None:
                    cells.append(cell)

            return cells

        def ischeaper(newcost, nextcell):
            if nextcell not in cost_so_far:
                return True
            (nextcost, nextyaw) = cost_so_far[nextcell]
            return newcost < nextcost

        def anglebetween(src, target):
            (x1, y1) = src
            (x2, y2) = target
            return math.atan2(y2 - y1, x2 - y1)

        while not frontier.empty():
            _, currentcell = frontier.get()

            if currentcell == target:
                completePath = []
                while currentcell != src:
                    completePath.append(currentcell)
                    currentcell = came_from[currentcell]

                completePath.append(src)
                completePath.reverse()

                return completePath

            for nextCell in adjecentcellsfor(currentcell):
                (currentcost, currentyaw) = cost_so_far[currentcell]
                new_cost = currentcost + coststoreach(currentyaw, currentcell, nextCell)
                if ischeaper(new_cost, nextCell):
                    cost_so_far[nextCell] = (new_cost, anglebetween(currentcell, nextCell))
                    priority = new_cost + distance(nextCell, target)
                    frontier.put((id(nextCell), nextCell), priority)
                    came_from[nextCell] = currentcell

        return None
