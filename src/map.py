#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid

class RobotMap:

    def __init__(self, widthInCm, heightInCm, topic):
        self.widthInCm = widthInCm
        self.heightInCm = heightInCm

        datasize = widthInCm * heightInCm

        self.data = bytearray(datasize)
        for x in range(widthInCm):
            for y in range(heightInCm):
                self.data[y * self.heightInCm + x] = 0

        self.topic = topic

    def markAsNotOccupied(self, x, y, radius):
        if int(y * 100) > int(self.heightInCm / 2):
            rospy.loginfo("x = %s, y = %s is NOT in the scope of the map!", x, y)
            return
        if int(x * 100) > int(self.heightInCm / 2):
            rospy.loginfo("x = %s, y = %s is NOT in the scope of the map!", x, y)
            return

        rospy.loginfo("Marking x = %s, y = %s as not occupied", x, y)
        robotPositionY = int(self.widthInCm / 2 + (y * 100))
        robotPositionX = int(self.heightInCm / 2 + (x * 100))
        for xc in range(int(radius * 2)):
            relxc = int(xc - radius)
            for yc in range(int(radius * 2)):
                relyc = int(yc - radius)
                if (relxc * relxc + relyc * relyc <= radius * radius):
                    pointX = robotPositionX + relxc
                    pointY = robotPositionY + relyc
                    self.data[pointY * self.heightInCm + pointX] = 100

    def publish(self):
        grid = OccupancyGrid()
        grid.info.resolution = 0.01
        grid.info.width = self.widthInCm
        grid.info.height = self.heightInCm
        grid.info.origin.position.x = - self.heightInCm / 2.0 * grid.info.resolution
        grid.info.origin.position.y = - self.widthInCm / 2.0 * grid.info.resolution
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        grid.data = list(self.data)

        self.topic.publish(grid)

