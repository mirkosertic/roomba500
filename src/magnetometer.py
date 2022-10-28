#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16
from sensor_msgs.msg import MagneticField

from HMC5883L import HMC5883L

class Magnetometer:

    def __init__(self):
        self.bus = None
        self.deviceaddress = 0x68
        self.magneticfieldpub = None
        self.magneticfieldframe = None
        self.hmc5883l = None

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

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling magnetometer..")
        while not rospy.is_shutdown():

            (x, y, z) = self.hmc5883l.axes()

            msg = MagneticField()
            msg.header.frame_id = self.magneticfieldframe
            msg.header.stamp = rospy.Time.now()

            if x is not None:
                msg.magnetic_field.x = x
            if y is not None:
                msg.magnetic_field.y = y
            if z is not None:
                msg.magnetic_field.z = z

            self.magneticfieldpub.publish(msg)

            rate.sleep()

        rospy.loginfo('Magnetometer terminated.')


if __name__ == '__main__':
    try:
        magnetometer = Magnetometer()
        magnetometer.start()
    except rospy.ROSInterruptException:
        pass

