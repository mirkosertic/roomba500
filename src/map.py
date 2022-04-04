#!/usr/bin/env python3

import enum
import math

import rospy
import numpy as np

from priorityqueue import PriorityQueue


class GridCellStatus(enum.Enum):
    UNKNOWN = 1
    OCCUPIED = 2
    FREE = 3


class GridMovementDirection(enum.Enum):
    RIGHT = 1
    TOP = 2
    LEFT = 3
    BOTTOM = 4


class BoustrophedonSpan:

    def __init__(self, start):
        self.cells = []
        self.cells.append(start)
        self.neighbors = []

    def append(self, cell):
        self.cells.append(cell)

    def appendNeighbors(self, neighbors):
        self.neighbors.extend(neighbors)


class MapGridCell:

    def __init__(self, centerx, centery, size, status):
        self.centerx = centerx
        self.centery = centery
        self.size = size

        # Pre-Compute the bounding box to speed up containment test
        halfsize = self.size / 2.0
        self.topleftx = self.centerx - halfsize
        self.toplefty = self.centery + halfsize
        self.bottomrightx = self.centerx + halfsize
        self.bottomrighty = self.centery - halfsize

        self.topleft = None
        self.top = None
        self.topright = None
        self.left = None
        self.right = None
        self.bottomleft = None
        self.bottom = None
        self.bottomright = None
        self.status = status

    def distanceTo(self, othercell):
        dx = othercell.centerx - self.centerx
        dy = othercell.centery - self.centery
        return math.sqrt(dx * dx + dy * dy)

    def isCovering(self, position):
        return position.x >= self.topleftx and position.x <= self.bottomrightx and position.y >= self.bottomrighty and position.y <= self.toplefty

    def updateStatusFrom(self, griddata, scanwidthinmeters, occupancythreshold):
        topleftx = self.centerx - scanwidthinmeters / 2
        toplefty = self.centery + scanwidthinmeters / 2

        bottomrightx = self.centerx + scanwidthinmeters / 2
        bottomrighty = self.centery - scanwidthinmeters / 2

        topleftxoc = int((topleftx - griddata.info.origin.position.x) / griddata.info.resolution)
        topleftyoc = int((toplefty - griddata.info.origin.position.y) / griddata.info.resolution)

        bottomrightxoc = int((bottomrightx - griddata.info.origin.position.x) / griddata.info.resolution)
        bottomrightyoc = int((bottomrighty - griddata.info.origin.position.y) / griddata.info.resolution)

        for y in range(bottomrightyoc, topleftyoc, 1):
            for x in range(topleftxoc, bottomrightxoc, 1):
                # Clip to valid coordinates
                if x >= 0 and x < griddata.info.width and y >= 0 and y < griddata.info.height:
                    value = griddata.data[y * griddata.info.width + x]
                    if value < 0:
                        # If one cell is unknown(-1), the status of the whole cell is unknown
                        self.status = GridCellStatus.UNKNOWN
                        return

        # We only check the center of the cell, as due to costmap inflation this also covers if it is reachable or not.
        xcenter = int((self.centerx - griddata.info.origin.position.x) / griddata.info.resolution)
        ycenter = int((self.centery - griddata.info.origin.position.y) / griddata.info.resolution)
        if griddata.data[ycenter * griddata.info.width + xcenter] > occupancythreshold:
            self.status = GridCellStatus.OCCUPIED
        else:
            self.status = GridCellStatus.FREE


class NavigationMap:

    def __init__(self, gridcellwidthinmeters, scanwidthinmeters, occupancythreshold):
        self.gridcellwidthinmeters = gridcellwidthinmeters
        self.scanwidthinmeters = scanwidthinmeters
        self.occupancythreshold = occupancythreshold
        self.cells = []
        self.currentmap =  None

    def isNewMap(self, griddata):
        if self.currentmap is None:
            return True

        if self.currentmap.info.width != griddata.info.width:
            return True

        if self.currentmap.info.height != griddata.info.height:
            return True

        if self.currentmap.info.resolution != griddata.info.resolution:
            return True

        if self.currentmap.info.origin.position.x != griddata.info.origin.position.x:
            return True

        if self.currentmap.info.origin.position.y != griddata.info.origin.position.y:
            return True

        return False

    def updateCellStatusFrom(self, griddata):
        self.currentmap = griddata
        for cell in self.cells:
            cell.updateStatusFrom(griddata, self.scanwidthinmeters, self.occupancythreshold)

    def initOrUpdateFromOccupancyGrid(self, griddata):

        rospy.loginfo("Scanning current map with grid size %s m", str(self.gridcellwidthinmeters))

        if not self.isNewMap(griddata):
            self.updateCellStatusFrom(griddata)
            return

        if self.currentmap is not None:
            rospy.loginfo("Changes in map position, size or resolution are currently not supported!")
            return

        origin = griddata.info.origin
        width = griddata.info.width
        height = griddata.info.height

        mapwidthinmeters = width * griddata.info.resolution
        mapheightinmeters = height * griddata.info.resolution

        squaresperrow = len(np.arange(0, mapwidthinmeters, self.gridcellwidthinmeters))

        yoffset = 0
        for y in np.arange(0, mapheightinmeters, self.gridcellwidthinmeters):
            xoffset = 0
            for x in np.arange(0, mapwidthinmeters, self.gridcellwidthinmeters):
                centerx = origin.position.x + x + (self.gridcellwidthinmeters / 2)
                centery = origin.position.y + y + (self.gridcellwidthinmeters / 2)

                cell = MapGridCell(centerx, centery, self.gridcellwidthinmeters, GridCellStatus.UNKNOWN)

                # We connect the cells
                if xoffset > 0:
                    # Connect left cell
                    left = self.cells[len(self.cells) - 1]
                    cell.left = left
                    left.right = cell

                if yoffset > 0:
                    # Bottom cell
                    bottom = self.cells[len(self.cells) - squaresperrow]
                    bottom.top = cell
                    cell.bottom = bottom

                if yoffset > 0 and xoffset > 0:
                    # Bottom left
                    bottomleft = self.cells[len(self.cells) - squaresperrow - 1]
                    bottomleft.topright = cell
                    cell.bottomleft = bottomleft

                if yoffset > 0 and xoffset < squaresperrow - 1:
                    # Bottom right
                    bottomright = self.cells[len(self.cells) - squaresperrow + 1]
                    bottomright.topleft = cell
                    cell.bottomright = bottomright

                self.cells.append(cell)

                xoffset = xoffset + 1

            yoffset = yoffset + 1

        self.updateCellStatusFrom(griddata)

    def nearestCellCovering(self, position):
        nearestCell = None
        nearestDistance = None
        for cell in self.cells:
            if cell.isCovering(position):
                dx = position.x - cell.centerx
                dy = position.y - cell.centery
                distance = math.sqrt(dx * dx + dy * dy)
                if nearestCell is None or distance < nearestDistance:
                    nearestCell = cell
                    nearestDistance = distance

        return nearestCell

    def pathlength(self, path):
        current = None
        length = 0
        for cell in path:
            if current is None:
                current = cell
            else:
                length = length + current.distanceTo(cell)
                current = cell

        return length

    def startCoverageBoustrophedon(self, src):
        # Boustrophedon cell decomposition

        def isFreeCell(cell):
            return cell is not None and cell.status == GridCellStatus.FREE

        def scanVertical(cell):
            # Search the top cell
            foundspans = []
            start = current
            while start.top is not None:
                start = start.top

            currentspan = None
            while start.bottom is not None:
                if isFreeCell(start):
                    if currentspan is None:
                        currentspan = BoustrophedonSpan(start)
                        foundspans.append(currentspan)
                    else:
                        currentspan.append(start)
                else:
                    currentspan = None

                start = start.bottom

            return foundspans

        spanstocover = []

        # Scan to the right
        current = src
        initialspans = None
        prevspans = None

        rospy.loginfo("Scanning to the right...")
        while current.right is not None:
            foundspans = scanVertical(current)
            spanstocover.extend(foundspans)
            if initialspans is None:
                initialspans = foundspans

            # TODO: Register neighbors in a bidirectional way

            prevspans = foundspans
            current = current.right

        # Scan to the left
        rospy.loginfo("Scanning to the left...")
        prevspans = initialspans
        current = src.left
        while current is not None and current.left is not None:
            foundspans = scanVertical(current)

            # TODO: Register neighbors in a bidirectional way

            spanstocover.extend(foundspans)
            current = current.left
            prevspans = foundspans

        path = []

        currentspan = None
        downmovement = True

        # Find the first span covering
        for span in spanstocover:
            if src in span.cells:
                currentspan = span

        while len(spanstocover) > 0:

            rospy.loginfo("Spans to cover %s", len(spanstocover))

            if downmovement:
                path.extend(currentspan.cells)
            else:
                path.extend(currentspan.cells[::-1])

            spanstocover.remove(currentspan)

            # Find the next possible span
            current = path[len(path) - 1]

            currentspan = None
            nearestdistance = .0
            nearestpath = None
            for span in spanstocover:
                possiblepath = self.findPath(current, span.cells[0])
                if possiblepath is None:
                    rospy.loginfo("Span is not reachable!")
                    spanstocover.remove(span)
                else:
                    d = self.pathlength(possiblepath)
                    if currentspan is None or d < nearestdistance:
                        currentspan = span
                        nearestdistance = d
                        nearestpath = possiblepath

                if possiblepath is not None:
                    possiblepath = self.findPath(current, span.cells[len(span.cells) - 1])
                    if possiblepath is not None:
                        d = self.pathlength(possiblepath)
                        if currentspan is None or d < nearestdistance:
                            currentspan = span
                            nearestdistance = d
                            nearestpath = possiblepath

            if currentspan is None:
                if len(spanstocover) == 0:
                    rospy.loginfo("Path seems to be complete")
                    return self.compress(path)

                rospy.loginfo("Cannot find nearest span")
                return self.compress(path)

            distanceTop = current.distanceTo(currentspan.cells[0])
            distanceBottom = current.distanceTo(currentspan.cells[len(currentspan.cells) - 1])
            downmovement = distanceTop < distanceBottom

            for index, cell in enumerate(nearestpath):
                if index > 0 and index < len(nearestpath) - 1:
                    path.append(cell)

        pass

    def compress(self, path):
        result = []
        for i, cell in enumerate(path):
            if i == 0 or i == len(path) - 1:
                # We have to keep the first and the last point of the path
                result.append(cell)
            else:
                # We check the trajectory from the previous point to the current
                prevCell = path[i - 1]
                currCell = cell
                nextCell = path[i + 1]

                tprev = math.atan2(currCell.centery - prevCell.centery, currCell.centerx - prevCell.centerx)
                tnext = math.atan2(nextCell.centery - currCell.centery, nextCell.centerx - currCell.centerx)

                if abs(tnext - tprev) > math.pi / 8:  # Change greater than 22.5 degrees
                    result.append(cell)

        return result

    def findPath(self, src, target):

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put((id(src), src), 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        def heuristicscore(src, target):
            dx = target.centerx - src.centerx
            dy = target.centery - src.centery
            #return math.sqrt(dx * dx + dy * dy)
            return abs(dx) + abs(dy)

        def adjecentcellsfor(src):
            cells = []

            def freeCell(cell):
                return cell is not None and cell.status == GridCellStatus.FREE

            if freeCell(src.top):
                cells.append(src.top)
            if freeCell(src.bottom):
                cells.append(src.bottom)
            if freeCell(src.left):
                cells.append(src.left)
            if freeCell(src.right):
                cells.append(src.right)

            if freeCell(src.top) and freeCell(src.right) and freeCell(src.topright):
                cells.append(src.topright)
            if freeCell(src.bottom) and freeCell(src.right) and freeCell(src.bottomright):
                cells.append(src.bottomright)
            if freeCell(src.top) and freeCell(src.left) and freeCell(src.topleft):
                cells.append(src.topleft)
            if freeCell(src.bottom) and freeCell(src.left) and freeCell(src.bottomleft):
                cells.append(src.bottomleft)

            return cells

        while not frontier.empty():
            _, currentcell = frontier.get()

            if currentcell == target:
                completePath = []
                while currentcell != src:
                    completePath.append(currentcell)
                    currentcell = came_from[currentcell]

                completePath.append(src)
                completePath.reverse()

                return self.compress(completePath)

            for nextCell in adjecentcellsfor(currentcell):
                new_cost = cost_so_far[currentcell] + heuristicscore(currentcell, nextCell)
                if nextCell not in cost_so_far or new_cost < cost_so_far[nextCell]:
                    cost_so_far[nextCell] = new_cost
                    priority = new_cost + heuristicscore(nextCell, target)
                    frontier.put((id(nextCell), nextCell), priority)
                    came_from[nextCell] = currentcell

        return None

    def toDebugImage(self):
        image = np.zeros((self.currentmap.info.height, self.currentmap.info.width, 3), np.uint8)
        image[:] = [0, 0, 0]

        for y in range(0, self.currentmap.info.height, 1):
            for x in range(0, self.currentmap.info.width, 1):
                value = self.currentmap.data[y * self.currentmap.info.width + x]
                if value == -1:
                    color = [200, 200, 200]
                elif value > self.occupancythreshold:
                    color = [0, 0, 0]
                else:
                    color = [255, 255, 255]

                image[self.currentmap.info.height - y - 1, x] = color

        for cell in self.cells:

            debugx = int((cell.centerx - self.currentmap.info.origin.position.x) / self.currentmap.info.resolution)
            debugy = int((cell.centery - self.currentmap.info.origin.position.y) / self.currentmap.info.resolution)

            if debugx < self.currentmap.info.width and debugy < self.currentmap.info.height:
                if cell.status == GridCellStatus.FREE:
                    image[self.currentmap.info.height - debugy - 1, debugx] = [0, 255, 0]
                elif cell.status == GridCellStatus.OCCUPIED:
                    image[self.currentmap.info.height - debugy - 1, debugx] = [0, 0, 255]

        return image
