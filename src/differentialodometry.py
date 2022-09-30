#!/usr/bin/env python3

import math
import threading
import rospy

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
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

        self.encodererrorthreshold = 0
        self.angularvelocitythreshold = .0

        self.latestSpeedLeftWheelMillimeterPerSecond = 0
        self.latestSpeedRightWheelMillimeterPerSecond = 0

        self.recoverFromCollision = False
        self.collisionX = .0
        self.collisionY = .0
        self.collisionTime = None
        self.collisionRevertDistance = .0
        self.collisionRevertVelocity = .0
        self.collisionReleaseDelta = .0
        self.bumperpcdistance = .0
        self.bumperpcheight = .0

        self.lightBumperheight = .0

        self.lightBumperLeftpcdistance = .0
        self.lightBumperLeftpcangle = .0

        self.lightBumperFrontLeftpcdistance = .0
        self.lightBumperFrontLeftpcangle = .0

        self.lightBumperCenterLeftpcdistance = .0
        self.lightBumperCenterLeftpcangle = .0

        self.lightBumperCenterRightpcdistance = .0
        self.lightBumperCenterRightpcangle = .0

        self.lightBumperFrontRightpcdistance = .0
        self.lightBumperFrontRightpcangle = .0

        self.lightBumperRightpcdistance = .0
        self.lightBumperRightpcangle = .0

        self.bumperspub = None
        self.lightsensorspub = None

        self.publishtf = True

    def newShutdownCommand(self, data):
        self.syncLock.acquire()
        rospy.signal_shutdown('Shutdown requested')
        self.syncLock.release()

    def publishCmdVel(self, linearX, angularZ):

        forwardSpeedMillimetersPerSecond = linearX * 100.0 * 10.0

        speedLeftWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond - (angularZ * (self.robotWheelSeparationInCm / 2.0) * 10.0))
        speedRightWheelMillimeterPerSecond = int(forwardSpeedMillimetersPerSecond + (angularZ * (self.robotWheelSeparationInCm / 2.0) * 10.0))

        if speedLeftWheelMillimeterPerSecond != self.latestSpeedLeftWheelMillimeterPerSecond or speedRightWheelMillimeterPerSecond != self.latestSpeedRightWheelMillimeterPerSecond:

            rospy.loginfo('Received new cmd_vel command with linear-x = %s m/s and angular-z = %s rad/s', linearX, angularZ)
            rospy.loginfo("Commanding motors with left wheel speed = %f mm/s and right wheel speed = %f mm/s", speedLeftWheelMillimeterPerSecond, speedRightWheelMillimeterPerSecond)

            speedcommand = DiffMotorSpeeds()
            speedcommand.leftMillimetersPerSecond = speedLeftWheelMillimeterPerSecond
            speedcommand.rightMillimetersPerSecond = speedRightWheelMillimeterPerSecond
            self.diffmotorspeedspub.publish(speedcommand)

            self.publishOdometry(True, rospy.Time.now())

            self.targetvelx = linearX
            self.targetvelz = angularZ
            self.moving = linearX != .0 or angularZ != .0

            self.latestSpeedLeftWheelMillimeterPerSecond = speedLeftWheelMillimeterPerSecond
            self.latestSpeedRightWheelMillimeterPerSecond = speedRightWheelMillimeterPerSecond

    def newCmdVelCommand(self, data):
        self.syncLock.acquire()

        if not self.recoverFromCollision:
            self.publishCmdVel(data.linear.x, data.angular.z)

        self.syncLock.release()

    def publishOdometry(self, commit, eventtime):

        currenttime = eventtime.to_sec()

        deltaleft = self.leftencoder.getDelta()
        deltaright = self.rightencoder.getDelta()

        if abs(deltaleft) > self.encodererrorthreshold or abs(deltaright) > self.encodererrorthreshold:
            rospy.logwarn("Got large delta in wheel encoders with left = %s and right = %s. I will assume an error and skip this frame.", deltaleft, deltaright)
            self.referencetime = currenttime
            return None

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

        velx = deltatravel / deltatime if deltatime > 0 else .0
        velz = deltatheta / deltatime if deltatime > 0 else .0
        if abs(velz) < self.angularvelocitythreshold:
            velz = .0

        if not self.moving:
            newtheta = self.referencetheta
            deltax = .0
            deltay = .0
            commit = True
            velx = .0
            velz = .0

        # Publish odometry and transform
        q = quaternion_from_euler(0, 0, newtheta)

        if self.publishtf:
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

        return (odom.pose.pose.position.x, odom.pose.pose.position.y, newtheta)

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
            # Enable failsafe
            if not self.recoverFromCollision:
                rospy.loginfo("Starting low-level recovery after collision. Blocking further cmd_vel commands.")

                # We stop the motors
                self.publishCmdVel(.0, .0)

                # And commit the current position
                newposition = self.publishOdometry(True, data.stamp)
                if newposition is not None:
                    # We mark the point from where we started
                    (x, y, theta) = newposition
                    self.recoverFromCollision = True
                    self.collisionTime = data.stamp
                    self.collisionX = x
                    self.collisionY = y

                    # Publish collision point cloud
                    dx = math.cos(theta) * self.bumperpcdistance
                    dy = math.sin(theta) * self.bumperpcdistance

                    bumpersmsg = PointCloud()
                    bumpersmsg.header.stamp = rospy.Time.now()
                    bumpersmsg.header.frame_id = 'odom'
                    bumpersmsg.points.append(Point32(x + dx, y + dy, self.bumperpcheight))
                    self.bumperspub.publish(bumpersmsg)

                    # And we start to drive backwards
                    self.publishCmdVel(self.collisionRevertVelocity, .0)
            else:
                # we are in recovery mode, and the bumpers are pressed
                newposition = self.publishOdometry(False, data.stamp)
        else:
            newposition = self.publishOdometry(False, data.stamp)

        if newposition is not None and self.recoverFromCollision:
            # We are in failsafe mode
            (currentX, currentY, _) = newposition
            dx = self.collisionX - currentX
            dy = self.collisionY - currentY
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > self.collisionRevertDistance:
                rospy.loginfo("Traveled back far enough : %s m", distance)
                # We can stop movement here
                self.publishOdometry(True, data.stamp)
                self.publishCmdVel(.0, .0)
            else:
                rospy.loginfo("Recovery distance is : %s m", distance)

            # After some time we can release the recovery flag
            deltaTime = data.stamp - self.collisionTime
            if deltaTime.to_sec() > self.collisionReleaseDelta:
                rospy.loginfo("Releasing cmd_vel command block after %s seconds", deltaTime.to_sec())
                self.recoverFromCollision = False
                self.publishOdometry(True, data.stamp)
                self.publishCmdVel(.0, .0)
            else:
                rospy.loginfo("Waiting to release command block, delta time is %s", deltaTime.to_sec())

        if newposition is not None:
            (x, y, theta) = newposition

            lightsensorsmsg = PointCloud()
            lightsensorsmsg.header.stamp = rospy.Time.now()
            lightsensorsmsg.header.frame_id = 'odom'
            if data.lightBumperLeftStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperLeftpcangle)) * self.lightBumperLeftpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperLeftpcangle)) * self.lightBumperLeftpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))
            if data.lightBumperFrontLeftStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperFrontLeftpcangle)) * self.lightBumperFrontLeftpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperFrontLeftpcangle)) * self.lightBumperFrontLeftpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))
            if data.lightBumperCenterLeftStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperCenterLeftpcangle)) * self.lightBumperCenterLeftpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperCenterLeftpcangle)) * self.lightBumperCenterLeftpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))
            if data.lightBumperCenterRightStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperCenterRightpcangle)) * self.lightBumperCenterRightpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperCenterRightpcangle)) * self.lightBumperCenterRightpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))
            if data.lightBumperFrontRightStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperFrontRightpcangle)) * self.lightBumperFrontRightpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperFrontRightpcangle)) * self.lightBumperFrontRightpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))
            if data.lightBumperRightStat:
                # Publish collision point cloud
                dx = math.cos(theta + math.radians(self.lightBumperRightpcangle)) * self.lightBumperRightpcdistance
                dy = math.sin(theta + math.radians(self.lightBumperRightpcangle)) * self.lightBumperRightpcdistance
                lightsensorsmsg.points.append(Point32(x + dx, y + dy, self.lightBumperheight))

            self.lightsensorspub.publish(lightsensorsmsg)

        self.syncLock.release()

    def start(self):
        rospy.init_node('differentialodometry', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        self.encodererrorthreshold = int(rospy.get_param('~encodererrorthreshold', '500'))
        self.angularvelocitythreshold = float(rospy.get_param('~angularvelocitythreshold', '0.05'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        self.ticksPerCm = float(rospy.get_param('~ticksPerCm', '22.7157014'))
        self.robotWheelSeparationInCm = float(rospy.get_param('~robotWheelSeparationInCm', '22.86'))  # 22.56 is calculated 22.86 seems to fit well

        self.collisionRevertDistance = float(rospy.get_param('~collisionRevertDistance', '0.2'))
        self.collisionRevertVelocity = float(rospy.get_param('~collisionRevertVelocity', '-0.2'))
        self.collisionReleaseDelta = float(rospy.get_param('~collisionReleaseDelta', '1'))

        self.bumperpcdistance = float(rospy.get_param('~bumperpcdistance', '0.167'))
        self.bumperpcheight = float(rospy.get_param('~bumperpcheight', '0.01'))

        self.lightBumperheight = float(rospy.get_param('~lightBumperheight', '0.05'))

        self.lightBumperLeftpcdistance = float(rospy.get_param('~lightBumperLeftpcdistance', '0.2175'))
        self.lightBumperLeftpcangle = float(rospy.get_param('~lightBumperLeftpcangle', '70'))

        self.lightBumperFrontLeftpcdistance = float(rospy.get_param('~lightBumperFrontLeftpcdistance', '0.2775'))
        self.lightBumperFrontLeftpcangle = float(rospy.get_param('~lightBumperFrontLeftpcangle', '35'))

        self.lightBumperCenterLeftpcdistance = float(rospy.get_param('~lightBumperCenterLeftpcdistance', '0.2525'))
        self.lightBumperCenterLeftpcangle = float(rospy.get_param('~lightBumperCenterLeftpcangle', '10'))

        self.lightBumperCenterRightpcdistance = float(rospy.get_param('~lightBumperCenterRightpcdistance', '0.2525'))
        self.lightBumperCenterRightpcangle = float(rospy.get_param('~lightBumperCenterRightpcangle', '-10'))

        self.lightBumperFrontRightpcdistance = float(rospy.get_param('~lightBumperFrontRightpcdistance', '0.2775'))
        self.lightBumperFrontRightpcangle = float(rospy.get_param('~lightBumperFrontRightpcangle', '-35'))

        self.lightBumperRightpcdistance = float(rospy.get_param('~lightBumperRightpcdistance', '0.2175'))
        self.lightBumperRightpcangle = float(rospy.get_param('~lightBumperRightpcangle', '-70'))

        self.publishtf = bool(rospy.get_param('~publish_tf', 'True'))

        rospy.loginfo("Configured with ticksPerCm                       = %s ", self.ticksPerCm)
        rospy.loginfo("Configured with robotWheelSeparationInCm         = %s ", self.robotWheelSeparationInCm)
        rospy.loginfo("Configured with collisionRevertDistance          = %s ", self.collisionRevertDistance)
        rospy.loginfo("Configured with collisionRevertVelocity          = %s ", self.collisionRevertVelocity)
        rospy.loginfo("Configured with collisionReleaseDelta            = %s ", self.collisionReleaseDelta)

        rospy.loginfo("Configured with bumperpcdistance                 = %s ", self.bumperpcdistance)
        rospy.loginfo("Configured with bumperpcheight                   = %s ", self.bumperpcheight)

        rospy.loginfo("Configured with lightBumperheight                = %s ", self.lightBumperheight)

        rospy.loginfo("Configured with lightBumperLeftpcdistance        = %s ", self.lightBumperLeftpcdistance)
        rospy.loginfo("Configured with lightBumperLeftpcangle           = %s ", self.lightBumperLeftpcangle)

        rospy.loginfo("Configured with lightBumperFrontLeftpcdistance   = %s ", self.lightBumperFrontLeftpcdistance)
        rospy.loginfo("Configured with lightBumperFrontLeftpcangle      = %s ", self.lightBumperFrontLeftpcangle)

        rospy.loginfo("Configured with lightBumperCenterLeftpcdistance  = %s ", self.lightBumperCenterLeftpcdistance)
        rospy.loginfo("Configured with lightBumperCenterLeftpcangle     = %s ", self.lightBumperCenterLeftpcangle)

        rospy.loginfo("Configured with lightBumperCenterRightpcdistance = %s ", self.lightBumperCenterRightpcdistance)
        rospy.loginfo("Configured with lightBumperCenterRightpcangle    = %s ", self.lightBumperCenterRightpcangle)

        rospy.loginfo("Configured with lightBumperFrontRightpcdistance  = %s ", self.lightBumperFrontRightpcdistance)
        rospy.loginfo("Configured with lightBumperFrontRightpangle      = %s ", self.lightBumperFrontRightpcangle)

        rospy.loginfo("Configured with lightBumperRightpcdistance       = %s ", self.lightBumperRightpcdistance)
        rospy.loginfo("Configured with lightBumperRightpcangle          = %s ", self.lightBumperRightpcangle)

        rospy.loginfo("Configured with publish_tf                       = %s ", self.publishtf)

        self.diffmotorspeedspub = rospy.Publisher('cmd_motorspeeds', DiffMotorSpeeds, queue_size=10)
        self.odompub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bumperspub = rospy.Publisher('bumpers', PointCloud, queue_size=10)
        self.lightsensorspub = rospy.Publisher('lightsensors', PointCloud, queue_size=10)

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

