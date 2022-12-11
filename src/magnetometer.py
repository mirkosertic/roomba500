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
        self.magneticfieldframe = None
        self.hmc5883l = None
        self.odomframe = 'odom'

        self.hardironx = None
        self.hardirony = None
        self.xyratio = None
        self.offsetindegrees = None

    def newShutdownCommand(self, data):
        rospy.signal_shutdown('Shutdown requested')

    def start(self):
        rospy.init_node('magnetometer', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        self.magneticfieldframe = rospy.get_param('~magnetometer_frame', 'base_link')
        self.hardironx = float(rospy.get_param('~hardironx', '-1236.7402'))
        self.hardirony = float(rospy.get_param('~hardirony', '-461.8761'))
        self.xyratio = float(rospy.get_param('~xyratio', '1.01'))
        self.offsetindegrees = float(rospy.get_param('~offsetindegrees', '-1.08'))

        lograwdata = bool(rospy.get_param('~lograwdata', 'False'))

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

        calibrationfile = str(pathlib.Path(rospy.get_param('~configdirectory', '/tmp')).joinpath('magcalibration.yaml'))
        if os.path.exists(calibrationfile):
            rospy.loginfo("Reading calibration data from %s", calibrationfile)
            with open(calibrationfile, 'r') as stream:
                data = yaml.safe_load(stream)
                self.hardironx = float(data["hardironx"])
                self.hardirony = float(data["hardirony"])
                self.xyratio = float(data["xyratio"])
                self.offsetindegrees = float(data["offsetindegrees"])

                rospy.loginfo(' hardironx       = ' + str(self.hardironx))
                rospy.loginfo(' hardirony       = ' + str(self.hardirony))
                rospy.loginfo(' xyratio         = ' + str(self.xyratio))
                rospy.loginfo(' offsetindegrees = ' + str(self.offsetindegrees))

        raw_data_handle = None
        if lograwdata:
            raw_data_dump = str(pathlib.Path(rospy.get_param('~configdirectory', '/tmp')).joinpath('mag_out.csv'))
            rospy.loginfo("Logging raw data to %s", raw_data_dump)
            raw_data_handle = open(raw_data_dump, 'w')
            raw_data_handle.write('x,y,x_scaled,y_scaled\n')

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        try:
            while not rospy.is_shutdown():

                (x, y, z) = self.hmc5883l.axes()

                # Correct orientation of sensor
                x = -x
                y = -y

                if x is not None and y is not None:

                    currenttime = rospy.Time.now()

                    # Translate the data back to origin (0,0)
                    xtrans = x - self.hardironx
                    ytrans = y - self.hardirony
                    rotation = math.radians(self.offsetindegrees)

                    # Rotate and scale the data to shape it into a "square" form
                    xscaled = xtrans * math.cos(rotation) - ytrans * math.sin(rotation)
                    yscaled = (xtrans * math.sin(rotation) + ytrans * math.cos(rotation)) * self.xyratio

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

                rate.sleep()
        except Exception as e:
            rospy.logerr('Error shutting down node : %s', e)

        if lograwdata:
            raw_data_handle.flush()
            raw_data_handle.close()

        rospy.loginfo('Saving calibration data to %s', calibrationfile)
        with open(calibrationfile, "w") as outfile:
            data = dict(
                hardironx = self.hardironx,
                hardirony = self.hardirony,
                xyratio = self.xyratio,
                offsetindegrees = self.offsetindegrees
            )
            yaml.dump(data, outfile, default_flow_style=False)

        rospy.loginfo('Magnetometer terminated.')


if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass
