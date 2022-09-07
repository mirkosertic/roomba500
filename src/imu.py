#!/usr/bin/env python3

import threading
import rospy

from std_msgs.msg import Int16
from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry

from MPU6050 import MPU6050

class IMU:

    def __init__(self):
        self.syncLock = threading.Lock()
        self.bus = None
        self.deviceaddress = 0x68
        self.imupub = None
        self.imuframe = None
        self.mpu = None
        self.latestorientation = None
        self.latestacceleration = None
        self.packet_size = 0
        self.odompub = None

    def newShutdownCommand(self, data):
        self.syncLock.acquire()
        rospy.signal_shutdown('Shutdown requested')
        self.syncLock.release()

    def readOrientation(self):
        FIFO_count = self.mpu.get_FIFO_count()
        mpu_int_status = self.mpu.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < self.packet_size:
                FIFO_count = self.mpu.get_FIFO_count()
            
            FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
            accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
            orientation = self.mpu.DMP_get_quaternion_int16(FIFO_buffer).get_normalized()
            grav = self.mpu.DMP_get_gravity(orientation)

            self.latestorientation = orientation
            self.latestacceleration = accel

    def processROSMessage(self):
        #Read Accelerometer raw value
        acc_x, acc_y, acc_z = self.mpu.get_acceleration()

        #Read Gyroscope raw value
        gyro_x, gyro_y, gyro_z = self.mpu.get_rotation()

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

        if self.latestorientation:		
            o = msg.orientation
            o.x, o.y, o.z, o.w = self.latestorientation.x, self.latestorientation.y, self.latestorientation.z, self.latestorientation.w

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'map'
            odom.pose.pose.position.x = 0
            odom.pose.pose.position.y = 0
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = self.latestorientation.x, self.latestorientation.y, self.latestorientation.z, self.latestorientation.w

            odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.latestacceleration.x, self.latestorientation.y
            odom.twist.twist.angular.z = self.latestacceleration.z
            self.odompub.publish(odom)

        msg.linear_acceleration.x = ax # / 9.807
        msg.linear_acceleration.y = ay # / 9.807
        msg.linear_acceleration.z = az # / 9.807

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        self.imupub.publish(msg)

        # print ("Gx=%.2f" %gx, u'\u00b0'+ "/s", "\tgy=%.2f" %gy, u'\u00b0'+ "/s", "\tgz=%.2f" %gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %ax, "\tAy=%.2f g" %ay, "\tAz=%.2f g" %az)

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
        x_accel_offset = 1265
        y_accel_offset = -2221
        z_accel_offset = -230
        x_gyro_offset = -7
        y_gyro_offset = 8
        z_gyro_offset = 64

        enable_debug_output = True

        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset, z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset, enable_debug_output)

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Publishing IMU data
        self.imupub = rospy.Publisher('imu/data', Imu, queue_size=10)

        self.odompub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling MPU6050...")
        while not rospy.is_shutdown():

            self.readOrientation()
            self.processROSMessage()

            rate.sleep()

        rospy.loginfo('IMU terminated.')


if __name__ == '__main__':
    try:
        imu = IMU()
        imu.start()
    except rospy.ROSInterruptException:
        pass

