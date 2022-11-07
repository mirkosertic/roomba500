#!/usr/bin/env python3

import rospy
import math
import os
import pathlib

from std_msgs.msg import Int16
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from tf.transformations import quaternion_from_euler

from HMC5883L import HMC5883L

class Magnetometer:

    def __init__(self):
        self.bus = None
        self.deviceaddress = 0x68
        self.magneticfieldpub = None
        self.magneticfieldodompub = None
        self.magneticfieldframe = None
        self.hmc5883l = None
        self.odomframe = 'odom'
        self.debugpointcloudpub = None

    def newShutdownCommand(self, data):
        rospy.signal_shutdown('Shutdown requested')

    def start(self):
        rospy.init_node('magnetometer', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '10'))

        self.magneticfieldframe = rospy.get_param('~magnetometer_frame', 'base_link')

        rospy.loginfo("Polling magnetometer data with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # Init MPU6050
        rospy.loginfo("Initializing HMC5883L magnetometer")
        i2c_bus = 1
        device_address = 0x1E

        self.hmc5883l = HMC5883L(i2c_bus, device_address)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Publishing IMU data
        self.magneticfieldpub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

        self.magneticfieldodompub = rospy.Publisher('magnetometer/odom', Odometry, queue_size=10)

        self.debugpointcloudpub = rospy.Publisher('magnetometer/debugpoints', PointCloud, queue_size=10)

        minx = None
        miny = None
        maxx = None
        maxy = None

        debugdata = PointCloud()
        debugdata.header.frame_id = self.magneticfieldframe

        calibrationfile = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('magcalibration.txt'))
        if os.path.exists(calibrationfile):
            rospy.loginfo("Reading calibration data from %s", calibrationfile)
            handle = open(calibrationfile, 'r')
            minx = float(handle.readline())
            miny = float(handle.readline())
            maxx = float(handle.readline())
            maxy = float(handle.readline())

            rospy.loginfo(' minx = ' + str(minx))
            rospy.loginfo(' miny = ' + str(miny))
            rospy.loginfo(' maxx = ' + str(maxx))
            rospy.loginfo(' maxy = ' + str(maxy))

            handle.close()

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        while not rospy.is_shutdown():

            (x, y, z) = self.hmc5883l.axes()

            currenttime = rospy.Time.now()

            magmessage = MagneticField()
            magmessage.header.frame_id = self.magneticfieldframe
            magmessage.header.stamp = currenttime

            if x is not None:
                if maxx is None:
                    maxx = x
                else:
                    maxx = max(x, maxx)

                if minx is None:
                    minx = x
                else:
                    minx = min(x, minx)

                magmessage.magnetic_field.x = x
            if y is not None:
                if maxy is None:
                    maxy = y
                else:
                    maxy = max(y, maxy)

                if miny is None:
                    miny = y
                else:
                    miny = min(y, miny)

                magmessage.magnetic_field.y = y
            if z is not None:

                magmessage.magnetic_field.z = z

            self.magneticfieldpub.publish(magmessage)

            rospy.logdebug("minx = %s miny = %s, maxx = %s, maxy = %s",str(minx), str(miny), str(maxx), str(maxy))

            if x is not None and y is not None and maxx > minx and maxy > miny:
                odommessage = Odometry()
                odommessage.header.stamp = currenttime
                odommessage.header.frame_id = self.odomframe
                odommessage.child_frame_id = self.magneticfieldframe

                scalex = 2 / (maxx - minx)
                scaley = 2 / (maxy - miny)
                dx = x - minx
                dy = y - miny

                xscaled = -1 + (dx * scalex)
                yscaled = -1 + (dy * scaley)

                rospy.logdebug("x = %s, y=%s scaled to x1 = %s, y1 = %s", str(x), str(y), str(xscaled), str(yscaled))

                roll = 0
                pitch = 0
                yaw = math.atan2(xscaled, yscaled)
                q = quaternion_from_euler(roll, pitch, yaw)

                rospy.logdebug("Mag x = %s, y = %s, yaw = %s", x, y, yaw)

                odommessage.pose.pose.orientation.x = q[0]
                odommessage.pose.pose.orientation.y = q[1]
                odommessage.pose.pose.orientation.z = q[2]
                odommessage.pose.pose.orientation.w = q[3]

                self.magneticfieldodompub.publish(odommessage)

                debugdata.header.stamp = currenttime

                # Make sure memory is not exploding
                if len(debugdata.points) > 1000:
                    debugdata.points.pop(0)

                debugdata.points.append(Point32(scalex, scaley, 0))
                self.debugpointcloudpub.publish(debugdata)

            rate.sleep()

        rospy.loginfo('Saving calibration data to %s', calibrationfile)
        handle = open(calibrationfile, 'w')
        handle.write('{:.6f}'.format(minx) + '\n')
        handle.write('{:.6f}'.format(miny) + '\n')
        handle.write('{:.6f}'.format(maxx) + '\n')
        handle.write('{:.6f}'.format(maxy) + '\n')
        handle.flush()
        handle.close()

        rospy.loginfo('Magnetometer terminated.')


if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass

