#!/usr/bin/env python3
import math
import threading
import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from roomba500.msg import RoombaSensorFrame, DiffMotorSpeeds

class DifferentialOdometry:

    def __init__(self):
        self.syncLock = threading.Lock()

        self.ticksPerCm = None
        self.fullRotationInSensorTicks = None
        self.robotWheelRadiusInCm = None

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

        rospy.loginfo('Received new cmd_vel command with linear-x = %s m/s and angular-z = %s rad/s', forwardSpeedMetersPerSecond, rotationRadPerSecond)

        forwardSpeedMillimetersPerSecond = forwardSpeedMetersPerSecond * 100 * 10

        speedLeftWheelMillimeterPerSecond = forwardSpeedMillimetersPerSecond - (rotationRadPerSecond * self.robotWheelRadiusInCm)
        speedRightWheelMillimeterPerSecond = forwardSpeedMillimetersPerSecond + (rotationRadPerSecond * self.robotWheelRadiusInCm)

        rospy.loginfo("Commanding motors with left wheel speed = %f mm/s and right wheel speed = %f mm/s", speedLeftWheelMillimeterPerSecond, speedRightWheelMillimeterPerSecond)

        speedcommand = DiffMotorSpeeds()
        speedcommand.leftMillimetersPerSecond = speedLeftWheelMillimeterPerSecond
        speedcommand.rightMillimetersPerSecond = speedRightWheelMillimeterPerSecond
        self.diffmotorspeedspub.publish(speedcommand)

        self.syncLock.release()

    def newSensorFrame(self, data):
        self.syncLock.acquire()

        wheelEncoderLeft = data.wheelEncoderLeft
        wheelEncoderRight = data.wheelEncoderRight

        self.computeAndPublishOdometry()

        self.syncLock.release()

    def computeAndPublishOdometry(self):
        pass

    def start(self):
        rospy.init_node('differentialodometry', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.7157014'))
        self.fullRotationInSensorTicks = float(rospy.get_param('~fullRotationInSensorTicks', '1415'))

        fullRotationInCm = self.fullRotationInSensorTicks / self.ticksPerCm
        self.robotWheelRadiusInCm = fullRotationInCm / (math.pi * 2)

        rospy.loginfo("Configured with ticksPerCm                = %s ", self.ticksPerCm)
        rospy.loginfo("Configured with fullRotationInSensorTicks = %s ", self.fullRotationInSensorTicks)
        rospy.loginfo("Configured with robotWheelRadiusInCm      = %s ", self.robotWheelRadiusInCm)

        self.diffmotorspeedspub = rospy.Publisher('cmd_motorspeeds', DiffMotorSpeeds, queue_size=10)
        self.odompub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Handling for cmd_vel and sensor frame data
        rospy.Subscriber("cmd_vel", Twist, self.newCmdVelCommand)
        rospy.Subscriber("sensorframe", RoombaSensorFrame, self.newSensorFrame)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        rospy.loginfo('Listening for cmd_vel commands...')

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            rate.sleep()

        rospy.loginfo('DifferentialOdometry terminated.')

if __name__ == '__main__':
    try:
        odometry = DifferentialOdometry()
        odometry.start()
    except rospy.ROSInterruptException:
        pass
