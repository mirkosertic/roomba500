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

        self.leftencoder = None
        self.rightencoder = None

        self.referencex = 0
        self.referencey = 0
        self.referencetheta = 0
        self.referencetime = None

        self.sumdeltaleft = 0
        self.sumdeltaright = 0

        self.diffmotorspeedspub = None
        self.odompub = None
        self.transformbroadcaster = None

        self.odomframe = 'odom'
        self.baselinkframe = 'base_link'

        self.targetvelx = .0
        self.targetvelz = .0
        self.moving = False

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

        speedLeftWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond - (rotationRadPerSecond * (self.robotWheelSeparationInCm / 2.0) * 10.0))
        speedRightWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond + (rotationRadPerSecond * (self.robotWheelSeparationInCm / 2.0) * 10.0))

        rospy.loginfo("Commanding motors with left wheel speed = %f mm/s and right wheel speed = %f mm/s", speedLeftWheelMillimeterPerSecond, speedRightWheelMillimeterPerSecond)

        speedcommand = DiffMotorSpeeds()
        speedcommand.leftMillimetersPerSecond = speedLeftWheelMillimeterPerSecond
        speedcommand.rightMillimetersPerSecond = speedRightWheelMillimeterPerSecond
        self.diffmotorspeedspub.publish(speedcommand)

        self.publishOdometry(True, rospy.Time.now())

        self.targetvelx = data.linear.x
        self.targetvelz = data.angular.z
        self.moving = data.linear.x != .0 or data.angular.z != .0

        self.syncLock.release()

    def publishOdometry(self, commit, eventtime):

        currenttime = eventtime.to_sec()

        deltaleft = self.leftencoder.getDelta()
        deltaright = self.rightencoder.getDelta()

        if abs(deltaleft) > 500 or abs(deltaright) > 500:
            rospy.logerror("Got large delta in wheel encoders with left = %s and right = %s. I will assume an error and skip this frame.", deltaleft, deltaright)
            return

        self.sumdeltaleft += deltaleft
        self.sumdeltaright += deltaright

        if self.referencetime is None:
            self.referencetime = currenttime
            deltatime = 0
        else:
            deltatime = currenttime - self.referencetime

        deltaleftinm = self.sumdeltaleft / self.ticksPerCm / 100.0
        deltarightinm = self.sumdeltaright / self.ticksPerCm / 100.0

        sumofdistances = deltaleftinm + deltarightinm
        deltatravel = (sumofdistances) / 2
        deltatheta = (deltarightinm - deltaleftinm) / (self.robotWheelSeparationInCm / 100.0)

        if deltaleftinm == deltarightinm:
            # Movement in straight line, either forward or backward
            deltax = deltaleftinm * math.cos(self.referencetheta)
            deltay = deltaleftinm * math.sin(self.referencetheta)
        elif abs(sumofdistances) < 10 and ((deltaleftinm > 0 and deltarightinm < 0) or (deltaleftinm < 0 and deltarightinm > 0)):
            # Rotation on the spot
            deltax = .0
            deltay = .0
        else:
            # Some rotation involved
            radius = deltatravel / deltatheta

            # Find the instantaneous center of curvature (ICC)
            iccx = self.referencex - radius * math.sin(self.referencetheta)
            iccy = self.referencey + radius * math.cos(self.referencetheta)

            deltax = math.cos(deltatheta) * (self.referencex - iccx) - math.sin(deltatheta) * (self.referencey - iccy) + iccx - self.referencex
            deltay = math.sin(deltatheta) * (self.referencex - iccx) + math.cos(deltatheta) * (self.referencey - iccy) + iccy - self.referencey

        newtheta = (self.referencetheta + deltatheta) % (2 * math.pi)

        # Update velocity after some time
        #
        # TODO: Disabled due to odometry corruption.
        #
        #if deltatime > 0.1 and deltaleft  != 0 and deltaright != 0:
        #    self.targetvelx = deltatravel / deltatime
        #    self.targetvelz = deltatheta / deltatime
        #    rospy.loginfo("Estimated velocity is linear-x = %s m/s and angular-z = %s rad/s", self.targetvelx, self.targetvelz)

        velx = deltatravel / deltatime if deltatime > 0 else .0
        velz = deltatheta / deltatime if deltatime > 0 else .0

        if not self.moving:
            newtheta = self.referencetheta
            deltax = .0
            deltay = .0
            commit = True
            velx = .0
            velz = .0

        # Publish odometry and transform
        q = quaternion_from_euler(0, 0, newtheta)
        self.transformbroadcaster.sendTransform(
            (self.referencex + deltax, self.referencey + deltay, 0),
            (q[0], q[1], q[2], q[3]),
            eventtime,
            self.baselinkframe,
            self.odomframe
        )

        odom = Odometry()
        odom.header.stamp = eventtime
        odom.header.frame_id = self.odomframe
        odom.child_frame_id = self.baselinkframe
        odom.pose.pose.position.x = self.referencex + deltax
        odom.pose.pose.position.y = self.referencey + deltay
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.targetvelx
        odom.twist.twist.angular.z = self.targetvelz

        odom.twist.twist.linear.x = velx
        odom.twist.twist.angular.z = velz

        if commit is True:
            self.referencex += deltax
            self.referencey += deltay
            self.referencetheta = newtheta
            self.sumdeltaleft = 0
            self.sumdeltaright = 0
            self.referencetime = currenttime

        self.odompub.publish(odom)

    def newSensorFrame(self, data):
        self.syncLock.acquire()

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

        if data.bumperLeft or data.bumperRight or data.wheeldropLeft or data.wheeldropRight:
            self.publishOdometry(True, data.stamp)
        else:
            self.publishOdometry(False, data.stamp)

        self.syncLock.release()

    def start(self):
        rospy.init_node('differentialodometry', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.7157014'))
        self.robotWheelSeparationInCm = float(rospy.get_param('~robotWheelSeparationInCm', '22.86'))  # 22.56 is calculated 22.86 seems to fit well

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

        rospy.loginfo('Listening for cmd_vel commands and sensor data to create odometry...')

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            rate.sleep()

        rospy.loginfo('DifferentialOdometry terminated.')


if __name__ == '__main__':
    try:
        # Code adapted from https://github.com/merose/diff_drive
        odometry = DifferentialOdometry()
        odometry.start()
    except rospy.ROSInterruptException:
        pass

