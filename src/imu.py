#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Int16
from sensor_msgs.msg import Imu

from MPU6050 import MPU6050

class IMU:

    def __init__(self):
        self.bus = None
        self.deviceaddress = 0x68
        self.imupub = None
        self.imuframe = None
        self.mpu = None
        self.latestorientation = None
        self.latestacceleration = None
        self.latestgyro = None
        self.packet_size = 0

        self.linearaccgainx = 0.0
        self.linearaccgainy = 0.0
        self.linearaccgainz = 0.0

        self.angularvelgainx = 0.0
        self.angularvelgainy = 0.0
        self.angularvelgainz = 0.0

    def newShutdownCommand(self, data):
        rospy.signal_shutdown('Shutdown requested')

    def readOrientationPull(self):

        #
        # Reset the FIFO, wait for an interrupt and read the whole packet
        # Everything else yields to somehow corrupt packets at some point. The
        # FIFO has a size of 1024 bytes, and the packet size is 42, which means
        # that there is a remainder in case of a mostly full buffer for the last
        # packet, which will be corrupted.
        self.mpu.reset_FIFO()

        FIFO_count = self.mpu.get_FIFO_count()
        while FIFO_count < self.packet_size and not rospy.is_shutdown():
            FIFO_count = self.mpu.get_FIFO_count()

        if rospy.is_shutdown():
            return False

        FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
        accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
        gyro = self.mpu.DMP_get_gyro_int16(FIFO_buffer)

        orientation_raw = self.mpu.DMP_get_quaternion(FIFO_buffer)
        orientation = orientation_raw.get_normalized()
        grav = self.mpu.DMP_get_gravity(orientation_raw)
        grav_normalized = self.mpu.DMP_get_gravity(orientation)

        validvalue = True

        if self.latestorientation is not None:
            current_roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(orientation, grav)
            latest_roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(self.latestorientation, grav)

            dx = current_roll_pitch_yaw.x - latest_roll_pitch_yaw.x
            dy = current_roll_pitch_yaw.y - latest_roll_pitch_yaw.y
            dz = current_roll_pitch_yaw.z - latest_roll_pitch_yaw.z

            if abs(dx) > 20 or abs(dy) > 20 or abs(dz) > 20:
                print('Ignoring linear measurement as dx = ' + str(dx) + " dy = " + str(dy) + " dz = " + str(dz))
                validvalue = False

            gyroerrorthreshold = 50 * 16.4

            if abs(gyro.x) > gyroerrorthreshold or abs(gyro.y) > gyroerrorthreshold or abs(gyro.z) > gyroerrorthreshold:
                print('Ignoring angular measurement as gyrox = ' + str(gyro.x) + " gyroy = " + str(gyro.y) + " gyroz = " + str(gyro.x))
                validvalue = False

        self.latestorientation = orientation
        self.latestacceleration = self.mpu.DMP_get_linear_accel(accel, grav)
        self.latestgyro = gyro


        # Example Output:
        # RAW Z Accel = 7517 Gravity Z = 0.975479856133461 Linear Accel Z without Gravity = -474.1309814453125
        # RAW Z Accel = 7509 Gravity Z = 0.975466538220644 Linear Accel Z without Gravity = -482.0218811035156
        # RAW Z Accel = 7516 Gravity Z = 0.975479856133461 Linear Accel Z without Gravity = -475.1309814453125
        # RAW Z Accel = 7510 Gravity Z = 0.975479856133461 Linear Accel Z without Gravity = -481.1309814453125
        # RAW Z Accel = 7512 Gravity Z = 0.975479856133461 Linear Accel Z without Gravity = -479.1309814453125
        # RAW Z Accel = 7519 Gravity Z = 0.9755083061754704 Linear Accel Z without Gravity = -472.3640441894531
        # DMP seems to be running with 8192 as a multiplier

        print('RAW Z Accel = ' + str(accel.z) + " Gravity Z = " + str(grav.z) + " Gravity Norm. Z = " + str(grav_normalized.z) + " Linear Accel Z without Gravity = " + str(self.latestacceleration.z))

        return validvalue

    def processROSMessage(self):
        msg = Imu()
        msg.header.frame_id = self.imuframe
        msg.header.stamp = rospy.Time.now()

        o = msg.orientation
        o.x, o.y, o.z, o.w = self.latestorientation.x, self.latestorientation.y, self.latestorientation.z, self.latestorientation.w
        # Convert g to m/s^2
        # DMP values should have a sensitivity of 16384 for 2g range.
        # However, the DMP seems to be at 4g range, so we have to use 8192 as the sensitivity.
        dmplinearmultiplier = 16384.0
        dmplinearmultiplier = 8192.0

        msg.linear_acceleration.x = self.linearaccgainx + (self.latestacceleration.x / dmplinearmultiplier * 9.80665)
        msg.linear_acceleration.y = self.linearaccgainy + (self.latestacceleration.y / dmplinearmultiplier * 9.80665)
        msg.linear_acceleration.z = self.linearaccgainz + (self.latestacceleration.z / dmplinearmultiplier * 9.80665)

        # Convert degrees/sec to rad/sec
        # The 16.4 / 10 constant is strange :
        #   16.4 is the sensitivity for each measurement with a 2000 deg/s resolution, but the DMP
        #   values seems to be scaled by the DMP sample rate, so 10 = 2000(the resolution) divided by 200 (which is
        #   the sample rate)
        msg.angular_velocity.x = self.angularvelgainx + (self.latestgyro.x * math.pi / 180.0 * 16.4 / 10.0)
        msg.angular_velocity.y = self.angularvelgainy + (self.latestgyro.y * math.pi / 180.0 * 16.4 / 10.0)
        msg.angular_velocity.z = self.angularvelgainz + (self.latestgyro.z * math.pi / 180.0 * 16.4 / 10.0)

        self.imupub.publish(msg)


    def start(self):
        rospy.init_node('imu', anonymous=True)
        pollingRateInHertz = int(rospy.get_param('~pollingRateInHertz', '20'))

        self.imuframe = rospy.get_param('~imu_frame', 'base_link')

        rospy.loginfo("Polling IMU data with %s hertz", pollingRateInHertz)
        rate = rospy.Rate(pollingRateInHertz)

        # Init MPU6050
        rospy.loginfo("Initializing MPU6050 IMU")
        i2c_bus = 1
        device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        x_accel_offset = 1055
        y_accel_offset = -1830
        z_accel_offset = -300
        x_gyro_offset = -7
        y_gyro_offset = 8
        z_gyro_offset = 62

        # Old values
        self.linearaccgainx = -0.115
        self.linearaccgainy = 0.08
        self.linearaccgainz = 0.583

        self.angularvelgainx = 0.007503
        self.angularvelgainy = 0.025122
        self.angularvelgainz = 0.056413

        calibrationmode = True
        calibrationsamples = 0
        calibrationmaxsamples = 500

        self.linearaccgainx = 0
        self.linearaccgainy = 0
        self.linearaccgainz = 0

        self.angularvelgainx = 0
        self.angularvelgainy = 0
        self.angularvelgainz = 0

        enable_debug_output = True

        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset, z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset, enable_debug_output)

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()

        # Handling for administrative shutdowns
        rospy.Subscriber("shutdown", Int16, self.newShutdownCommand)

        # Publishing IMU data
        self.imupub = rospy.Publisher('imu/data', Imu, queue_size=10)

        # Processing the sensor polling in an endless loop until this node shuts down
        rospy.loginfo("Polling MPU6050 DMP...")
        while not rospy.is_shutdown():

            if self.readOrientationPull():

                if calibrationmode:
                    calibrationsamples = calibrationsamples + 1

                    rospy.loginfo('Calibration. Taking sample #' + str(calibrationsamples))

                    # DMP values should have a sensitivity of 16384 for 2g range.
                    # However, the DMP seems to be at 4g range, so we have to use 8192 as the sensitivity.
                    dmplinearmultiplier = 16384.0
                    dmplinearmultiplier = 8192.0

                    self.linearaccgainx = self.linearaccgainx + (self.latestacceleration.x / dmplinearmultiplier * 9.80665)
                    self.linearaccgainy = self.linearaccgainy + (self.latestacceleration.y / dmplinearmultiplier * 9.80665)
                    self.linearaccgainz = self.linearaccgainz + (self.latestacceleration.z / dmplinearmultiplier * 9.80665)

                    self.angularvelgainx = self.angularvelgainx + (self.latestgyro.x * math.pi / 180.0 * 16.4 / 10.0)
                    self.angularvelgainy = self.angularvelgainy + (self.latestgyro.y * math.pi / 180.0 * 16.4 / 10.0)
                    self.angularvelgainz = self.angularvelgainz + (self.latestgyro.z * math.pi / 180.0 * 16.4 / 10.0)

                    if calibrationsamples >= calibrationmaxsamples:
                        calibrationmode = False

                        self.linearaccgainx = -(self.linearaccgainx / calibrationsamples)
                        self.linearaccgainy = -(self.linearaccgainy / calibrationsamples)
                        self.linearaccgainz = -(self.linearaccgainz / calibrationsamples)

                        self.angularvelgainx = -(self.angularvelgainx / calibrationsamples)
                        self.angularvelgainy = -(self.angularvelgainy / calibrationsamples)
                        self.angularvelgainz = -(self.angularvelgainz / calibrationsamples)

                        rospy.loginfo('Calibration finished:')
                        rospy.loginfo(' linearaccgainx = ' + str(self.linearaccgainx))
                        rospy.loginfo(' linearaccgainy = ' + str(self.linearaccgainy))
                        rospy.loginfo(' linearaccgainz = ' + str(self.linearaccgainz))
                        rospy.loginfo(' angularvelgainx = ' + str(self.angularvelgainx))
                        rospy.loginfo(' angularvelgainy = ' + str(self.angularvelgainy))
                        rospy.loginfo(' angularvelgainz = ' + str(self.angularvelgainz))

                else:
                    self.processROSMessage()

            rate.sleep()

        rospy.loginfo('IMU terminated.')


if __name__ == '__main__':
    try:
        imu = IMU()
        imu.start()
    except rospy.ROSInterruptException:
        pass

