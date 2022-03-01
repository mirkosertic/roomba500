#!/usr/bin/env python3

import rospy

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

    def drive(self, speed, angularZRotation):
        twistMsg = Twist()
        twistMsg.linear.x = speed
        twistMsg.linear.y = .0
        twistMsg.linear.z = .0
        twistMsg.angular.x = .0
        twistMsg.angular.y = .0
        twistMsg.angular.z = angularZRotation

        rospy.logdebug("Publishing twist %s", twistMsg)

        self.cmdvelPublisher.publish(twistMsg)
