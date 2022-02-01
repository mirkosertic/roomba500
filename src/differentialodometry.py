#!/usr/bin/env python3

import math
import threading
import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster

from roomba500.msg import RoombaSensorFrame, DiffMotorSpeeds

from encoder import Encoder

class DifferentialOdometry:

    def __init__(self):
        self.syncLock = threading.Lock()

        self.ticksPerCm = None
        self.robotWheelSeparationInCm = None

        self.lastsensortime = None
        self.leftencoder = None
        self.rightencoder = None

        self.x = 0
        self.y = 0
        self.theta = 0
        self.xvel = 0
        self.yvel = 0
        self.thetavel = 0

        self.diffmotorspeedspub = None
        self.odompub = None
        self.transformbroadcaster = None

    def newShutdownCommand(self, data):
        self.syncLock.acquire()
        rospy.signal_shutdown('Shutdown requested')
        self.syncLock.release()

    def newCmdVelCommand(self, data):
        self.syncLock.acquire()

        forwardSpeedMetersPerSecond = data.linear.x
        rotationRadPerSecond = data.angular.z

        rospy.loginfo('Received new cmd_vel command with linear-x = %s m/s and angular-z = %s rad/s', forwardSpeedMetersPerSecond, rotationRadPerSecond)

        forwardSpeedMillimetersPerSecond = forwardSpeedMetersPerSecond * 100.0 * 10.0

        speedLeftWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond - (rotationRadPerSecond * (self.robotWheelSeparationInCm  / 2.0) * 10.0))
        speedRightWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond + (rotationRadPerSecond * (self.robotWheelSeparationInCm / 2.0) * 10.0))

        rospy.loginfo("Commanding motors with left wheel speed = %f mm/s and right wheel speed = %f mm/s", speedLeftWheelMillimeterPerSecond, speedRightWheelMillimeterPerSecond)

        speedcommand = DiffMotorSpeeds()
        speedcommand.leftMillimetersPerSecond = speedLeftWheelMillimeterPerSecond
        speedcommand.rightMillimetersPerSecond = speedRightWheelMillimeterPerSecond
        self.diffmotorspeedspub.publish(speedcommand)

        self.syncLock.release()

    def newSensorFrame(self, data):
        self.syncLock.acquire()

        currenttime = rospy.get_time()

        if self.leftencoder is None:
            self.leftencoder = Encoder()
            self.leftencoder.initCount(data.wheelEncoderLeft)
        else:
            self.leftencoder.update(data.wheelEncoderLeft)

        if self.rightencoder is None:
            self.rightencoder = Encoder()
            self.rightencoder.initCount(data.wheelEncoderRight)
        else:
            self.rightencoder.update(data.wheelEncoderRight)

        deltaleft = self.leftencoder.getDelta()
        deltaright = self.rightencoder.getDelta()

        if self.lastsensortime is None:
            self.lastsensortime = currenttime
            deltatime = 0
        else:
            deltatime = currenttime - self.lastsensortime
            self.lastsensortime = currenttime

        deltaleftinm = deltaleft / self.ticksPerCm / 100.0
        deltarightinm = deltaright / self.ticksPerCm / 100.0

        deltatravel = (deltarightinm + deltaleftinm) / 2
        deltatheta = (deltarightinm - deltaleftinm) / (self.robotWheelSeparationInCm / 100.0)

        if deltaleftinm == deltarightinm:
            # Movement in straight line
            deltax = deltaleftinm * math.cos(self.theta)
            deltay = deltaleftinm * math.sin(self.theta)
        else:
            # Some rotation involved
            radius = deltatravel / deltatheta

            # Find the instantaneous center of curvature (ICC)
            iccx = self.x - radius * math.sin(self.theta)
            iccy = self.y + radius * math.cos(self.theta)

            deltax = math.cos(deltatheta) * (self.x - iccx) - math.sin(deltatheta) * (self.y - iccy) + iccx - self.x
            deltay = math.sin(deltatheta) * (self.x - iccx) + math.cos(deltatheta) * (self.y - iccy) + iccy - self.y

        self.x += deltax
        self.y += deltay
        self.theta = (self.theta + deltatheta) % (2 * math.pi)
        self.xvel = deltatravel / deltatime if deltatime > 0 else 0.
        self.yvel = 0
        self.thetavel = deltatheta / deltatime if deltatime > 0 else 0.

        now = rospy.Time.now()

        # Publish odometry and transform
        q = quaternion_from_euler(0, 0, self.theta)
        self.transformbroadcaster.sendTransform(
            (self.x, self.y, 0),
            (q[0], q[1], q[2], q[3]),
            now,
            'base_link',
            'odom'
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.xvel
        odom.twist.twist.angular.z = self.thetavel
        self.odompub.publish(odom)

        self.syncLock.release()

    def computeAndPublishOdometry(self):
        pass

    def start(self):
        rospy.init_node('differentialodometry', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.7157014'))
        self.robotWheelSeparationInCm = float(rospy.get_param('~robotWheelSeparationInCm', '22.86'))  # 32.56 is calculated 22.86

        rospy.loginfo("Configured with ticksPerCm                = %s ", self.ticksPerCm)
        rospy.loginfo("Configured with robotWheelSeparationInCm  = %s ", self.robotWheelSeparationInCm)

        self.diffmotorspeedspub = rospy.Publisher('cmd_motorspeeds', DiffMotorSpeeds, queue_size=10)
        self.odompub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.transformbroadcaster = TransformBroadcaster()

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
