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

import smbus

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

class Accelerometer:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.bus = None
        self.deviceaddress = 0x68

    def newShutdownCommand(self, data):
        self.syncLock.acquire()
        rospy.signal_shutdown('Shutdown requested')
        self.syncLock.release()

    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.deviceaddress, addr)
        low = self.bus.read_byte_data(self.deviceaddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    def start(self):
        rospy.init_node('accelerometer', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        rospy.loginfo("Checking system state with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # Init MPU6050
        self.bus = smbus.SMBus(1)

        #write to sample rate register
        self.bus.write_byte_data(self.deviceaddress, SMPLRT_DIV, 7)
        #Write to power management register
        self.bus.write_byte_data(self.deviceaddress, PWR_MGMT_1, 1)
        #Write to Configuration register
        self.bus.write_byte_data(self.deviceaddress, CONFIG, 0)
        #Write to Gyro configuration register
        self.bus.write_byte_data(self.deviceaddress, GYRO_CONFIG, 24)
        #Write to interrupt enable register
        self.bus.write_byte_data(self.deviceaddress, INT_ENABLE, 1)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            #Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            #Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0

            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0

            print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            rate.sleep()

        rospy.loginfo('Accelerometer terminated.')


if __name__ == '__main__':
    try:
        accel = Accelerometer()
        accel.start()
    except rospy.ROSInterruptException:
        pass

