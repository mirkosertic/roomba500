#!/usr/bin/env python3

import threading
import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from roomba500.msg import RoombaSensorFrame, DiffMotorSpeeds

class DifferentialOdometry:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.transformlistener = None
        self.diffmotorspeedspub = None
        self.odompub = None

    def newShutdownCommand(self, data):
        self.syncLock.acquire()
        rospy.signal_shutdown('Shutdown requested')
        self.syncLock.release()

    def newCmdVelCommand(self, data):
        self.syncLock.acquire()

        forwardSpeedMetersPerSecond = data.linear.x
        rotationRadPerSecond = data.angular.z

        rospy.loginfo('Received new cmd_vel command with linear-x = %s m/second and angular-z = %s rad/second', forwardSpeedMetersPerSecond, rotationRadPerSecond)

        self.syncLock.release()

    def newSensorFrame(self, data):
        self.syncLock.acquire()

        wheelEncoderLeft = data.wheelEncoderLeft
        wheelEncoderRight = data.wheelEncoderRight

        self.syncLock.release()

    def computeAndPublishOdometry(self, data):
        pass

    def start(self):
        rospy.init_node('differentialodometry', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.diffmotorspeedspub = rospy.Publisher('cmd_motorspeeds', DiffMotorSpeeds, queue_size=10)
        self.odompub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Handling for cmd_vel and sensor frame data
        rospy.Subscriber("cmd_vel", Twist, self.newCmdVelCommand)
        rospy.Subscriber("sensorframe", RoombaSensorFrame, self.newSensorFrame)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            self.syncLock.acquire()
            self.computeAndPublishOdometry()
            self.syncLock.release()

            rate.sleep()

        rospy.loginfo('DifferentialOdometry terminated.')

if __name__ == '__main__':
    try:
        odometry = DifferentialOdometry()
        DifferentialOdometry.start()
    except rospy.ROSInterruptException:
        pass
