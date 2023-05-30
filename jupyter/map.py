from priorityqueue import PriorityQueue

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
    
    def findpath(self, src, target):

        # Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
        frontier = PriorityQueue()
        frontier.put((id(src), src), 0)

        came_from = {}
        cost_so_far = {}
        came_from[src] = None
        cost_so_far[src] = 0

        def heuristicscore(src, target):
            (x1, y1) = src
            (x2, y2) = target
            dx = x2 - x1
            dy = y2 - y1
            return abs(dx) + abs(dy)

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
                new_cost = cost_so_far[currentcell] + heuristicscore(currentcell, nextCell)
                if nextCell not in cost_so_far or new_cost < cost_so_far[nextCell]:
                    cost_so_far[nextCell] = new_cost
                    priority = new_cost + heuristicscore(nextCell, target)
                    frontier.put((id(nextCell), nextCell), priority)
                    came_from[nextCell] = currentcell

        return None
