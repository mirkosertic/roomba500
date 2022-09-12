#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Int16
from sensor_msgs.msg import Imu

from MPU6050 import MPU6050
from MPUConstants import MPUConstants as C
from Quaternion import XYZVector as V

class IMU:

    def __init__(self):
        self.bus = None
        self.deviceaddress = 0x68
        self.imupub = None
        self.imuframe = None
        self.mpu = None

        self.linearaccgainx = 0.0
        self.linearaccgainy = 0.0
        self.linearaccgainz = 0.0

        self.angularvelgainx = 0.0
        self.angularvelgainy = 0.0
        self.angularvelgainz = 0.0

    def newShutdownCommand(self, data):
        rospy.signal_shutdown('Shutdown requested')

    def start(self):
        rospy.init_node('imu', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '40'))

        self.imuframe = rospy.get_param('~imu_frame', 'map')

        rospy.loginfo("Polling IMU data with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # Init MPU6050
        rospy.loginfo("Initializing MPU6050 IMU")
        i2c_bus = 1
        device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        x_accel_offset = 1117
        y_accel_offset = -1857
        z_accel_offset = -298
        x_gyro_offset = -6
        y_gyro_offset = 8
        z_gyro_offset = 57

        self.linearaccgainx = -0.115
        self.linearaccgainy = 0.08
        self.linearaccgainz = 0.583

        self.angularvelgainx = 0.007503
        self.angularvelgainy = 0.025122
        self.angularvelgainz = 0.056413

        enable_debug_output = True

        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset, z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset, enable_debug_output)
        self.mpu.set_full_scale_gyro_range(C.MPU6050_GYRO_FS_250)
        self.mpu.set_full_scale_accel_range(C.MPU6050_ACCEL_FS_2)

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Publishing IMU data
        self.imupub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling MPU6050...")

        calibrationmode = True
        calibrationsamples = 0


        self.linearaccgainx = 0
        self.linearaccgainy = 0
        self.linearaccgainz = 0

        self.angularvelgainx = 0
        self.angularvelgainy = 0
        self.angularvelgainz = 0

        while not rospy.is_shutdown():

            accel_reading = self.mpu.get_acceleration()
            gyro_reading = self.mpu.get_rotation()

            linear_accel_x = accel_reading[0] / 16384.0 * 9.80665
            linear_accel_y = accel_reading[1] / 16384.0 * 9.80665
            linear_accel_z = accel_reading[2] / 16384.0 * 9.80665

            angular_vel_x = gyro_reading[0] / 131.0 * math.pi / 180.0
            angular_vel_y = gyro_reading[1] / 131.0 * math.pi / 180.0
            angular_vel_z = gyro_reading[2] / 131.0 * math.pi / 180.0

            if calibrationmode:

                calibrationsamples = calibrationsamples + 1

                rospy.loginfo('Calibration. Taking sample #' + str(calibrationsamples))

                self.linearaccgainx = self.linearaccgainx + linear_accel_x
                self.linearaccgainy = self.linearaccgainy + linear_accel_y
                self.linearaccgainz = self.linearaccgainz + linear_accel_z

                self.angularvelgainx = self.angularvelgainx + angular_vel_x
                self.angularvelgainy = self.angularvelgainy + angular_vel_y
                self.angularvelgainz = self.angularvelgainz + angular_vel_z

                if calibrationsamples >= 1000:
                    calibrationmode = False

                    self.linearaccgainx = self.linearaccgainx / calibrationsamples
                    self.linearaccgainy = self.linearaccgainy / calibrationsamples
                    self.linearaccgainz = self.linearaccgainz / calibrationsamples

                    self.angularvelgainx = self.angularvelgainx / calibrationsamples
                    self.angularvelgainy = self.angularvelgainy / calibrationsamples
                    self.angularvelgainz = self.angularvelgainz / calibrationsamples

                    rospy.loginfo('Calibration finished:')
                    rospy.loginfo(' linearaccgainx = ' + str(self.linearaccgainx))
                    rospy.loginfo(' linearaccgainy = ' + str(self.linearaccgainy))
                    rospy.loginfo(' linearaccgainz = ' + str(self.linearaccgainz))
                    rospy.loginfo(' angularvelgainx = ' + str(self.angularvelgainx))
                    rospy.loginfo(' angularvelgainy = ' + str(self.angularvelgainy))
                    rospy.loginfo(' angularvelgainz = ' + str(self.angularvelgainz))

            else:
                msg = Imu()
                msg.header.frame_id = self.imuframe
                msg.header.stamp = rospy.Time.now()

                # Convert g to m/s^2
                msg.linear_acceleration.x = self.linearaccgainx + linear_accel_x
                msg.linear_acceleration.y = self.linearaccgainy + linear_accel_y
                msg.linear_acceleration.z = self.linearaccgainz + linear_accel_z

                # Convert degrees/sec to rad/sec
                msg.angular_velocity.x = self.angularvelgainx + angular_vel_x
                msg.angular_velocity.y = self.angularvelgainy + angular_vel_y
                msg.angular_velocity.z = self.angularvelgainz + angular_vel_z

                self.imupub.publish(msg)

            rate.sleep()

        rospy.loginfo('IMU terminated.')


if __name__ == '__main__':
    try:
        imu = IMU()
        imu.start()
    except rospy.ROSInterruptException:
        pass

