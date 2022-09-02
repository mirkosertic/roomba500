#!/usr/bin/env python3

import threading
import rospy
import numpy as np

from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_about_axis

import smbus

# Code adapted from https://github.com/OSUrobotics/mpu_6050_driver/blob/master/scripts/imu_node.py

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

class IMU:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.bus = None
        self.deviceaddress = 0x68
        self.imupub = None
        self.imuframe = None

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

    def process(self):
        #Read Accelerometer raw value
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        ax = acc_x/16384.0
        ay = acc_y/16384.0
        az = acc_z/16384.0

        gx = gyro_x/131.0
        gy = gyro_y/131.0
        gz = gyro_z/131.0

        msg = Imu()
        msg.header.frame_id = self.imuframe
        msg.header.stamp = rospy.Time.now()

        accel = ax, ay, az
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        o = msg.orientation
        o.x, o.y, o.z, o.w = orientation

        msg.linear_acceleration.x = ax # / 9.807
        msg.linear_acceleration.y = ay # / 9.807
        msg.linear_acceleration.z = az # / 9.807

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        self.imupub.publish(msg)

        print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)

    def start(self):
        rospy.init_node('imu', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        self.imuframe = rospy.get_param('~imu_frame', 'imu_link')

        rospy.loginfo("Polling IMU data with %s hertz", pollingRateInHertz)
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

        # Publishing IMU data
        self.imupub = rospy.Publisher('imu/data', Imu)

        # Processing the sensor polling in an endless loop until this node shuts down
        while not rospy.is_shutdown():

            self.process()

            rate.sleep()

        rospy.loginfo('IMU terminated.')


if __name__ == '__main__':
    try:
        imu = IMU()
        imu.start()
    except rospy.ROSInterruptException:
        pass

