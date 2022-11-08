#!/usr/bin/env python3

import rospy
import math
import os
import pathlib

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
        self.minx = float(rospy.get_param('~initial_minx', '-601.68'))
        self.miny = float(rospy.get_param('~initial_miny', '-199.64'))
        self.maxx = float(rospy.get_param('~initial_maxx', '-79.12'))
        self.maxy = float(rospy.get_param('~initial_maxy', '283.36'))

        lograwdata = bool(rospy.get_param('~lograwdata', 'True'))

        hard_iron_bias_x = float(rospy.get_param('~hard_iron_bias_x', '244.6953822373436'))
        hard_iron_bias_y = float(rospy.get_param('~hard_iron_bias_y', '-478.3109347752146'))
        hard_iron_bias_z = float(rospy.get_param('~hard_iron_bias_z', '187.88851271580867'))

        soft_iron_bias_xx = float(rospy.get_param('~soft_iron_bias_xx', '0.0819028609649276'))
        soft_iron_bias_xy =  float(rospy.get_param('~soft_iron_bias_xy', '-0.08186652146555928'))
        soft_iron_bias_xz =  float(rospy.get_param('~soft_iron_bias_xz', '0.0052316581647013'))

        soft_iron_bias_yx =  float(rospy.get_param('~soft_iron_bias_yx', '-0.08186652146555924'))
        soft_iron_bias_yy =  float(rospy.get_param('~soft_iron_bias_yy', '0.08193966402754398'))
        soft_iron_bias_yz =  float(rospy.get_param('~soft_iron_bias_yz', '0.0004502243369786666'))

        soft_iron_bias_zx =  float(rospy.get_param('~soft_iron_bias_zx', '0.005231658164701299'))
        soft_iron_bias_zy =  float(rospy.get_param('~soft_iron_bias_zy', '0.0004502243369786652'))
        soft_iron_bias_zz =  float(rospy.get_param('~soft_iron_bias_zz', '0.2950141209373251'))

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

        calibrationfile = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('magcalibration.txt'))
        if os.path.exists(calibrationfile):
            rospy.loginfo("Reading calibration data from %s", calibrationfile)
            handle = open(calibrationfile, 'r')
            self.minx = float(handle.readline())
            self.miny = float(handle.readline())
            self.maxx = float(handle.readline())
            self.maxy = float(handle.readline())

            rospy.loginfo(' minx = ' + str(self.minx))
            rospy.loginfo(' miny = ' + str(self.miny))
            rospy.loginfo(' maxx = ' + str(self.maxx))
            rospy.loginfo(' maxy = ' + str(self.maxy))

            handle.close()

        raw_data_handle = None
        if lograwdata:
            raw_data_dump = str(pathlib.Path(rospy.get_param('~roomdirectory', '/tmp')).joinpath('mag_out.txt'))
            rospy.loginfo("Logging raw data to %s", raw_data_dump)
            raw_data_handle = open(raw_data_dump, 'w')

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        try:
            while not rospy.is_shutdown():

                (x, y, z) = self.hmc5883l.axes()

                if x is not None:
                    if self.maxx is None:
                        self.maxx = x
                    else:
                        self.maxx = max(x, self.maxx)

                    if self.minx is None:
                        self.minx = x
                    else:
                        self.minx = min(x, self.minx)

                if y is not None:
                    if self.maxy is None:
                        self.maxy = y
                    else:
                        self.maxy = max(y, self.maxy)

                    if self.miny is None:
                        self.miny = y
                    else:
                        self.miny = min(y, self.miny)

                rospy.logdebug("minx = %s miny = %s, maxx = %s, maxy = %s",str(self.minx), str(self.miny), str(self.maxx), str(self.maxy))

                if x is not None and y is not None: #and self.maxx > self.minx and self.maxy > self.miny:

                    z = 100

                    if lograwdata:
                        raw_data_handle.write('{:.6f}'.format(x) + ',')
                        raw_data_handle.write('{:.6f}'.format(y) + ',0\n')

                    currenttime = rospy.Time.now()

                    odommessage = Odometry()
                    odommessage.header.stamp = currenttime
                    odommessage.header.frame_id = self.odomframe
                    odommessage.child_frame_id = self.magneticfieldframe

                    # Implementation is here: https://github.com/nliaudat/magnetometer_calibration
                    xm_off = x - hard_iron_bias_x
                    ym_off = y - hard_iron_bias_y
                    zm_off = z - hard_iron_bias_z

                    xm_cal = xm_off * soft_iron_bias_xx + ym_off * soft_iron_bias_yx + zm_off * soft_iron_bias_zx
                    ym_cal = xm_off * soft_iron_bias_xy + ym_off * soft_iron_bias_yy + zm_off * soft_iron_bias_zy
                    zm_cal = xm_off * soft_iron_bias_xz + ym_off * soft_iron_bias_yz + zm_off * soft_iron_bias_zz

                    #scalex = 2 / (self.maxx - self.minx)
                    #scaley = 2 / (self.maxy - self.miny)
                    #dx = x - self.minx
                    #dy = y - self.miny

                    #xscaled = -1 + (dx * scalex)
                    #yscaled = -1 + (dy * scaley)

                    rospy.logdebug("x = %s, y=%s scaled to x1 = %s, y1 = %s", str(x), str(y), str(xm_cal), str(ym_cal))

                    magmessage = MagneticField()
                    magmessage.header.frame_id = self.magneticfieldframe
                    magmessage.header.stamp = currenttime
                    magmessage.magnetic_field.x = ym_cal
                    magmessage.magnetic_field.y = xm_cal
                    self.magneticfieldpub.publish(magmessage)

                    magmessageraw = MagneticField()
                    magmessageraw.header.frame_id = self.magneticfieldframe
                    magmessageraw.header.stamp = currenttime
                    magmessageraw.magnetic_field.x = y
                    magmessageraw.magnetic_field.y = x
                    self.magneticfieldpubraw.publish(magmessageraw)

                    roll = 0
                    pitch = 0
                    yaw = math.atan2(xm_cal, ym_cal)
                    q = quaternion_from_euler(roll, pitch, yaw)

                    rospy.logdebug("Mag x = %s, y = %s, yaw = %s", x, y, yaw)

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
        handle = open(calibrationfile, 'w')
        handle.write('{:.6f}'.format(self.minx) + '\n')
        handle.write('{:.6f}'.format(self.miny) + '\n')
        handle.write('{:.6f}'.format(self.maxx) + '\n')
        handle.write('{:.6f}'.format(self.maxy) + '\n')
        handle.flush()
        handle.close()

        rospy.loginfo('Magnetometer terminated.')


if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass

