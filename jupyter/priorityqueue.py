#!/usr/bin/env python3

import heapq


# Code taken from https://www.redblobgames.com/pathfinding/a-star/implementation.html
class PriorityQueue:

    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
