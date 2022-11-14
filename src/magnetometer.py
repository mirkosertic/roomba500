#!/usr/bin/env python3

import rospy
import math
import os
import pathlib
import yaml

from std_msgs.msg import Int16
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

from HMC5883L import HMC5883L

class Magnetometer:

    def __init__(self):
        self.bus = None
        self.deviceaddress = 0x68
        self.magneticfieldpub = None
        self.magneticfieldpubraw = None
        self.magneticfieldodompub = None
        self.magneticfieldframe = None
        self.hmc5883l = None
        self.odomframe = 'odom'

        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxy = None

    def newShutdownCommand(self, data):
        rospy.signal_shutdown('Shutdown requested')

    def start(self):
        rospy.init_node('magnetometer', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        self.magneticfieldframe = rospy.get_param('~magnetometer_frame', 'base_link')
        self.minx = float(rospy.get_param('~initial_minx', '-138.92'))
        self.miny = float(rospy.get_param('~initial_miny', '-73.6'))
        self.maxx = float(rospy.get_param('~initial_maxx', '327.52'))
        self.maxy = float(rospy.get_param('~initial_maxy', '340.4'))

        lograwdata = bool(rospy.get_param('~lograwdata', 'True'))

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
        self.magneticfieldpubraw = rospy.Publisher('imu/mag_raw', MagneticField, queue_size=10)

        self.magneticfieldodompub = rospy.Publisher('magnetometer/odom', Odometry, queue_size=10)

        calibrationfile = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('magcalibration.yaml'))
        if os.path.exists(calibrationfile):
            rospy.loginfo("Reading calibration data from %s", calibrationfile)
            with open(calibrationfile, 'r') as stream:
                data = yaml.safe_load(stream)
                self.minx = float(data["minx"])
                self.miny = float(data["miny"])
                self.maxx = float(data["maxx"])
                self.maxy = float(data["maxy"])

                rospy.loginfo(' minx = ' + str(self.minx))
                rospy.loginfo(' miny = ' + str(self.miny))
                rospy.loginfo(' maxx = ' + str(self.maxx))
                rospy.loginfo(' maxy = ' + str(self.maxy))

        raw_data_handle = None
        if lograwdata:
            raw_data_dump = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('mag_out.csv'))
            rospy.loginfo("Logging raw data to %s", raw_data_dump)
            raw_data_handle = open(raw_data_dump, 'w')

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        try:
            while not rospy.is_shutdown():

                (x, y, z) = self.hmc5883l.axes()

                if x is not None and y is not None:

                    currenttime = rospy.Time.now()

                    # This is just a simple hard iron compensation
                    # We move the center of the data area to x,y = (0,0) and scale
                    # it to be quadratic. The data seem to form a nice circle with
                    # some noise in it, so no need to do advanced ellipsoid fitting
                    # to get it in the right shape, so no further soft iron compensation.
                    width = self.maxx - self.minx
                    height = self.maxy - self.miny
                    xyratio = width / height

                    biasx = -(self.minx + self.maxx) / 2.0
                    biasy = -(self.miny + self.maxy) / 2.0

                    xscaled = biasx + x
                    yscaled = (biasy + y) * xyratio

                    rospy.logdebug("x = %s, y=%s scaled to x1 = %s, y1 = %s", str(x), str(y), str(xscaled), str(yscaled))

                    if lograwdata:
                        raw_data_handle.write('{:.6f}'.format(x) + ',')
                        raw_data_handle.write('{:.6f}'.format(y) + ',')
                        raw_data_handle.write('{:.6f}'.format(xscaled) + ',')
                        raw_data_handle.write('{:.6f}'.format(yscaled) + '\n')


                    magmessage = MagneticField()
                    magmessage.header.frame_id = self.magneticfieldframe
                    magmessage.header.stamp = currenttime
                    magmessage.magnetic_field.x = xscaled
                    magmessage.magnetic_field.y = yscaled
                    self.magneticfieldpub.publish(magmessage)

                    magmessageraw = MagneticField()
                    magmessageraw.header.frame_id = self.magneticfieldframe
                    magmessageraw.header.stamp = currenttime
                    magmessageraw.magnetic_field.x = x
                    magmessageraw.magnetic_field.y = y
                    self.magneticfieldpubraw.publish(magmessageraw)

                    odommessage = Odometry()
                    odommessage.header.stamp = currenttime
                    odommessage.header.frame_id = "base_link"
                    odommessage.child_frame_id = "base_link"

                    roll = 0
                    pitch = 0
                    yaw = math.atan2(yscaled, xscaled)
                    q = quaternion_from_euler(roll, pitch, yaw)

                    odommessage.pose.pose.orientation.x = q[0]
                    odommessage.pose.pose.orientation.y = q[1]
                    odommessage.pose.pose.orientation.z = q[2]
                    odommessage.pose.pose.orientation.w = q[3]

                    self.magneticfieldodompub.publish(odommessage)

                rate.sleep()
        except Exception as e:
            rospy.logerr('Error shutting down node : %s', e)

        if lograwdata:
            raw_data_handle.flush()
            raw_data_handle.close()

        rospy.loginfo('Saving calibration data to %s', calibrationfile)
        with open(calibrationfile, "w") as outfile:
            data = dict(
                minx = self.minx,
                miny = self.miny,
                maxx = self.maxx,
                maxy = self.maxy
            )
            yaml.dump(data, outfile, default_flow_style=False)

        rospy.loginfo('Magnetometer terminated.')


if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass
