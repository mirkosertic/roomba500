#!/usr/bin/env python3

import pathlib
import threading
import time

import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from roomba500.msg import RoombaSensorFrame
from std_msgs.msg import Int16


class RoombaLogger:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.odometrylogfile = None

        self.latestevent = ''
        self.positionx = .0
        self.positiony = .0
        self.rotation = .0

        self.velocityx = .0
        self.velocityy = .0
        self.velocityangular = .0

        self.wheelcoderleft = 0
        self.wheelcoderright = 0

        self.separator = ','

    def newShutdownCommand(self, data):
        rospy.logdebug("Received shutdown command")
        rospy.signal_shutdown('Shutdown requested')

    def newOdometry(self, data):
        self.syncLock.acquire()

        position = data.pose.pose.position
        orientation_q = data.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        velocity = data.twist.twist

        self.positionx = position.x
        self.positiony = position.y
        self.rotation = yaw

        self.velocityx = velocity.linear.x
        self.velocityy = velocity.linear.y
        self.velocityangular = velocity.angular.z

        self.writeLog()
        self.syncLock.release()

    def newCmdVelCommand(self, data):
        self.syncLock.acquire()

        self.latestevent = 'Linear x= ' + self.formatNumber(data.linear.x) + ' m/s  Angular z= ' + self.formatNumber(data.angular.z) + ' rad/s'

        self.writeLog()
        self.syncLock.release()

    def newRoombaSensorFrame(self, data):
        self.syncLock.acquire()

        self.wheelcoderleft = data.wheelEncoderLeft
        self.wheelcoderright = data.wheelEncoderRight

        self.writeLog()
        self.syncLock.release()

    def formatNumber(self, number):
        return '{:.3f}'.format(number)

    def writeLog(self):

        timestamp = round(time.time() * 1000)

        self.odometrylogfile.write(self.latestevent + self.separator + str(timestamp) + self.separator + self.formatNumber(self.positionx) + self.separator + self.formatNumber(self.positiony) + self.separator + self.formatNumber(self.rotation) + self.separator + self.formatNumber(self.velocityx) + self.separator + self.formatNumber(self.velocityx) + self.separator + self.formatNumber(self.velocityangular) + self.separator + self.formatNumber(self.wheelcoderleft) + self.separator + self.formatNumber(self.wheelcoderright) + '\n')
        self.odometrylogfile.flush()

        self.latestevent = ''

    def writeLogHeader(self):
        self.odometrylogfile.write('event' + self.separator + 'ms since start' + self.separator + 'positionx' + self.separator + 'positiony' + self.separator + 'rotationz' + self.separator + 'velocityx' + self.separator + 'velocityy' + self.separator + 'velocityz' + self.separator + 'wheelencoderleft' + self.separator + 'wheelencoderright' + '\n')
        self.odometrylogfile.flush()

    def start(self):
        rospy.init_node('roombalogger', anonymous=True)

        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '40'))
        rospy.loginfo('Checking system state with %s hertz', pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        odometrylogfilename = rospy.get_param('~roombalog', str(pathlib.Path(__file__).parent.resolve().parent.joinpath('maps', 'roombalog.txt')))

        rospy.loginfo('Writing odometry debug to %s', odometrylogfilename)
        self.odometrylogfile = open(odometrylogfilename, 'w')
        self.writeLogHeader()

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Consume data from these sources and aggregate them to a log
        rospy.Subscriber('odom', Odometry, self.newOdometry)
        rospy.Subscriber('cmd_vel', Twist, self.newCmdVelCommand)
        rospy.Subscriber('sensorframe', RoombaSensorFrame, self.newRoombaSensorFrame)

        rospy.loginfo('Starting main loop')
        while not rospy.is_shutdown():
            rate.sleep()

        self.odometrylogfile.close()

        rospy.loginfo('RoombaLogger terminated.')


if __name__ == '__main__':
    try:
        rl = RoombaLogger()
        rl.start()
    except rospy.ROSInterruptException:
        pass
