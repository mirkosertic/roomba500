#!/usr/bin/env python3

import rospy
import math
import time

from driver import Driver

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class State:

    def handleIMUMessage(self, message):
        pass

    def handleLifetick(self):
        return self

class NoOpState(State):

    def __init__(self):
        pass

class RotationSpeedTest(State):

    def __init__(self, driver):
        self.count = 0
        self.driver = driver
        self.averageangularvelz = .0
        self.testspeeds = [.3, .4, .5, .6, .8, -.3, -.4, -.5, -.6, -.8]

    def handleIMUMessage(self, message):
        self.averageangularvelz = (self.averageangularvelz + message.angular_velocity.z) / 2.0

    def logresults(self, expected):
        rospy.loginfo("Rotation test finished: expected speed : %s, average speed : %s", expected, self.averageangularvelz)

    def handleLifetick(self):
        if self.count < len(self.testspeeds):
            targetspeed = self.testspeeds[self.count]
            if self.count == 0:
                rospy.loginfo("Teststep : %s", self.count)
                self.driver.drive(.0, targetspeed)
                self.averageangularvelz = targetspeed
            else:
                expected = self.testspeeds[self.count - 1]
                self.logresults(expected)

                self.driver.drive(.0, .0)
                time.sleep(2)

                rospy.loginfo("Teststep : %s", self.count)
                self.driver.drive(.0, targetspeed)
                self.averageangularvelz = targetspeed

            self.count = self.count + 1
            return self
        else:
            expected = self.testspeeds[self.count - 1]
            self.logresults(expected)

        self.driver.drive(.0, .0)
        rospy.loginfo("Rotation test finished")
        return AccelerationTest(self.driver)

class AccelerationTest(State):

    def __init__(self, driver):
        self.count = 0
        self.driver = driver
        self.maxaccelx = None
        self.maxaccely = None
        self.testspeeds = [.3, .4]

    def resetMeasurements(self):
        self.maxaccelx = None
        self.maxaccely = None

    def handleIMUMessage(self, message):
        if self.maxaccelx is None:
            self.maxaccelx = abs(message.linear_acceleration.x)
        else:
            self.maxaccelx = max(self.maxaccelx, abs(message.linear_acceleration.x))

        if self.maxaccely is None:
            self.maxaccely= abs(message.linear_acceleration.y)
        else:
            self.maxaccely = max(self.maxaccely, abs(message.linear_acceleration.y))

    def logresults(self, expected):
        rospy.loginfo("Acceleration test finished: expected speed x : %s, abs accelx : %s, abs accely = %s, degrees = %s", expected, self.maxaccelx, self.maxaccely, math.degrees(math.atan2(self.maxaccely, self.maxaccelx)))

    def handleLifetick(self):
        if self.count < len(self.testspeeds):
            targetspeed = self.testspeeds[self.count]
            if self.count == 0:
                rospy.loginfo("Teststep : %s", self.count)
                self.driver.drive(targetspeed, .0)
                self.resetMeasurements()
            else:
                expected = self.testspeeds[self.count - 1]
                self.logresults(expected)

                self.driver.drive(.0, .0)
                time.sleep(2)

                rospy.loginfo("Teststep : %s", self.count)
                self.driver.drive(targetspeed, .0)
                self.resetMeasurements()

            self.count = self.count + 1
            return self
        else:
            expected = self.testspeeds[self.count - 1]
            self.logresults(expected)

        self.driver.drive(.0, .0)
        rospy.loginfo("Acceleration test finished")
        return NoOpState()

class Calibration:

    def __init__(self):
        self.state = None
        self.driver = None

    def imuMessage(self, message):
        self.state.handleIMUMessage(message)

    def start(self):
        rospy.init_node('imucalibration', anonymous=True)
        rate = rospy.Rate(.5)

        self.driver = Driver(rospy.Publisher('cmd_vel', Twist, queue_size=10))

        rospy.Subscriber("imu/data", Imu, self.imuMessage)

        self.state = NoOpState()

        rospy.loginfo("Running calibration")
        counter = 0
        while not rospy.is_shutdown():
            self.state = self.state.handleLifetick()

            counter = counter + 1
            if (counter == 1):
                self.state = RotationSpeedTest(self.driver)

            rate.sleep()


        rospy.loginfo("Calibration finished")

if __name__ == '__main__':
    try:
        calibration = Calibration()
        calibration.start()
    except rospy.ROSInterruptException:
        pass

