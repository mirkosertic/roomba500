#!/usr/bin/env python3

import cv2
import math
import heapq
from typing import TypeVar, Dict, List, Optional


Location = TypeVar('Location')

img = cv2.imread('../maps/turtlebot3.png')
print(img.shape)
imgHeight, imgWidth, _ = img.shape


def checksquare(img, x, y, width, height, threshold):
    for xa in range(width - 1):
        for ya in range(height - 1):
            (b, g, r) = img[y + ya, x + xa]
            grayscale = (int(r) + int(g) + int(b)) / 3
            if grayscale < threshold:
                return True

    return False


#
# Marching Squares: we check which areas of the map are occupied or not
#
squareWidth = 20
squareHeight = 20

# Calculate the grid from the map
gridSizeX = int(imgWidth / squareWidth) + 1
gridSizeY = int(imgHeight / squareHeight) + 1
mapFromOccupancyGrid = [[-1 for x in range(gridSizeX)] for y in range(gridSizeY)]

print("Map size is", imgWidth, imgHeight)
print("Grid size is", gridSizeX, gridSizeY)

for x in range(0, imgWidth - squareWidth, squareWidth):
    for y in range(0, imgHeight - squareHeight, squareHeight):
        xInOccupancyGrid = int(x / squareWidth)
        yInOccupancyGrid = int(y / squareHeight)
        if not checksquare(img, x, y, squareWidth, squareHeight, 220):
            mapFromOccupancyGrid[yInOccupancyGrid][xInOccupancyGrid] = 0


#
# Now we do have a map that can be used for navigation
#
def validcoordinate(x, y):
    global gridSizeX, gridSizeY

    if x < 0 or y < 0 or x >= gridSizeX or y >= gridSizeY:
        return False

    return True


def visit(x, y, currentheading, iteration):
    global mapFromOccupancyGrid, img, gridSizeX, gridSizeY

    # We mark the visited point
    if mapFromOccupancyGrid[y][x] != 0:
        return

    mapFromOccupancyGrid[y][x] = iteration

    # Now we calculate the score for the surrounding movement possibilities
    def score(x1, y1):
        totalscore = 0
        if not validcoordinate(x1, y1):
            return -1, (x1, y1)

        def subcheck(x1a, y1a, score):
            if not validcoordinate(x1a, y1a):
                return score + 1

            if mapFromOccupancyGrid[y1a][x1a] != 0:
                return score + 1

            return score

        # Top row
        totalscore = subcheck(x1 - 1, y1 - 1, totalscore)
        totalscore = subcheck(x1, y1 - 1, totalscore)
        totalscore = subcheck(x1 + 1, y1 - 1, totalscore)
        # Left and right
        totalscore = subcheck(x1 - 1, y1, totalscore)
        totalscore = subcheck(x1 + 1, y1, totalscore)
        # Bottom row
        totalscore = subcheck(x1 - 1, y1 + 1, totalscore)
        totalscore = subcheck(x1, y1 + 1, totalscore)
        totalscore = subcheck(x1 + 1, y1 + 1, totalscore)

        return totalscore, (x1, y1)

    bestfit = None
    bestheading = None
    lastfound = 0

    def updatebestfit(x1, y1, bestfit, bestheading, lastfound, currentheading, movementheading):
        if validcoordinate(x1, y1):
            if mapFromOccupancyGrid[y1][x1] == 0:
                (s, pos) = score(x1, y1)
                if currentheading == movementheading:
                    s = s * 1

                if s > lastfound:
                    return pos, movementheading, s

        return bestfit, bestheading, lastfound

    # Top row
    (bestfit, bestheading, lastfound) = updatebestfit(x - 1, y - 1, bestfit, bestheading, lastfound, currentheading, 1)
    (bestfit, bestheading, lastfound) = updatebestfit(x, y - 1, bestfit, bestheading, lastfound, currentheading, 2)
    (bestfit, bestheading, lastfound) = updatebestfit(x + 1, y - 1, bestfit, bestheading, lastfound, currentheading, 3)
    # Left and right
    (bestfit, bestheading, lastfound) = updatebestfit(x - 1, y, bestfit, bestheading, lastfound, currentheading, 4)
    (bestfit, bestheading, lastfound) = updatebestfit(x + 1, y, bestfit, bestheading, lastfound, currentheading, 5)
    # Bottom row
    (bestfit, bestheading, lastfound) = updatebestfit(x - 1, y + 1, bestfit, bestheading, lastfound, currentheading, 6)
    (bestfit, bestheading, lastfound) = updatebestfit(x, y + 1, bestfit, bestheading, lastfound, currentheading, 7)
    (bestfit, bestheading, lastfound) = updatebestfit(x + 1, y + 1, bestfit, bestheading, lastfound, currentheading, 8)

    if iteration > 200:
        return

    if bestfit is not None:
        (x, y) = bestfit
        visit(x, y, bestheading, iteration + 1)

# visit(10, 2, 5, 1)


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item: Location, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> Location:
        return heapq.heappop(self.elements)[1]


def heuristicscore(src: Location, target: Location):
    x1, y1 = src
    x2, y2 = target
    dx = x2 - x1
    dy = y2 - y1
    #return math.sqrt(dx * dx + dy * dy)
    return abs(dx) + abs(dy)


def adjecentcellsfor(cell: Location):
    global mapFromOccupancyGrid

    result = []
    x, y = cell

    def appendifvalid(x, y):
        if validcoordinate(x, y):
            if mapFromOccupancyGrid[y][x] != -1:
                result.append((x, y))

    appendifvalid(x - 1, y - 1)
    appendifvalid(x, y - 1)
    appendifvalid(x + 1, y - 1)
    appendifvalid(x - 1, y)
    appendifvalid(x + 1, y)
    appendifvalid(x - 1, y + 1)
    appendifvalid(x, y + 1)
    appendifvalid(x + 1, y + 1)

    return result


def findpath(src, target):
    global mapFromOccupancyGrid

    # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
    frontier = PriorityQueue()
    frontier.put(src, 0)

    came_from: Dict[Location, Optional[Location]] = {}
    cost_so_far: Dict[Location, Float] = {}
    came_from[src] = None
    cost_so_far[src] = 0

    while not frontier.empty():
        currentcell: Location = frontier.get()

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


path = findpath((10, 2), (7 + 5 - 5, 15))

if path is not None:
    for index, element in enumerate(path):
        x, y = element
        mapFromOccupancyGrid[y][x] = index + 1


for y in range(gridSizeY):
    for x in range(gridSizeX):
        print('{0:4d}'.format(mapFromOccupancyGrid[y][x]), end='')
    print('')

# And draw in some debug data
for x in range(gridSizeX - 1):
    for y in range(gridSizeY - 1):
        color = [0, 255, 0]
        if mapFromOccupancyGrid[y][x] < 0:
            color = [0, 0, 255]
        if mapFromOccupancyGrid[y][x] > 0:
            color = [255, 0, 0]

        leftX = x * squareWidth
        leftY = y * squareHeight
        for xa in range(squareWidth - 1):
            img[leftY, leftX + xa] = color
            img[leftY + squareHeight - 1, leftX + xa] = color

        for ya in range(squareHeight - 1):
            img[leftY + ya, leftX] = color
            img[leftY + ya, leftX + squareWidth - 1] = color

cv2.imwrite('../maps/turtlebot3_marching.png', img)
