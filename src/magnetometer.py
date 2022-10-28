#!/usr/bin/env python3

import rospy
import math

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
        self.magneticfieldodompub = None
        self.magneticfieldframe = None
        self.hmc5883l = None
        self.odomframe = 'odom'

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

        self.magneticfieldodompub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        while not rospy.is_shutdown():

            (x, y, z) = self.hmc5883l.axes()

            currenttime = rospy.Time.now()

            magmessage = MagneticField()
            magmessage.header.frame_id = self.magneticfieldframe
            magmessage.header.stamp = currenttime

            if x is not None:
                magmessage.magnetic_field.x = x
            if y is not None:
                magmessage.magnetic_field.y = y
            if z is not None:
                magmessage.magnetic_field.z = z

            self.magneticfieldpub.publish(magmessage)

            if x is not None and y is not None:
                odommessage = Odometry()
                odommessage.header.stamp = currenttime
                odommessage.header.frame_id = self.odomframe
                odommessage.child_frame_id = self.magneticfieldframe

                roll = 0
                pitch = 0
                yaw = math.atan2(y, x)
                q = quaternion_from_euler(roll, pitch, yaw)

                rospy.loginfo("Mag x = %s, y = %s, yaw = %s", x, y, yaw)

                odommessage.pose.pose.orientation.x = q[0]
                odommessage.pose.pose.orientation.y = q[1]
                odommessage.pose.pose.orientation.z = q[2]
                odommessage.pose.pose.orientation.w = q[3]

                self.magneticfieldodompub.publish(odommessage)

            rate.sleep()

        rospy.loginfo('Magnetometer terminated.')
-113 138

if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass
