#!/usr/bin/env python

from geometry_msgs.msg import Twist

class Driver:

    def __init__(self, cmdvelPublisher):
        self.cmdvelPublisher = cmdvelPublisher

    def stop(self):
        twistMsg = Twist()
        twistMsg.linear.x = .0
        twistMsg.linear.y = .0
        twistMsg.linear.z = .0
        twistMsg.angular.x = .0
        twistMsg.angular.y = .0
        twistMsg.angular.z = .0

        self.cmdvelPublisher.publish(twistMsg)

    def rotateZ(self, speed):
        twistMsg = Twist()
        twistMsg.linear.x = .0
        twistMsg.linear.y = .0
        twistMsg.linear.z = .0
        twistMsg.angular.x = .0
        twistMsg.angular.y = .0
        twistMsg.angular.z = speed

        self.cmdvelPublisher.publish(twistMsg)
