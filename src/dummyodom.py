#!/usr/bin/env python3

import rospy
import tf

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3, PoseStamped

class DummyOdom:

    def __init__(self):
        return

    def start(self):
        rospy.init_node('dummyodom', anonymous=True)
        publishRateRateInHertz = int(rospy.get_param('~publishRateRateInHertz', '20'))

        rospy.loginfo("Publishing dummy odometry with %s hertz", publishRateRateInHertz)
        rate = rospy.Rate(publishRateRateInHertz)

        # We consume odometry here
        odomPub = rospy.Publisher("odom", Odometry, queue_size = 10)
        transformBroadcaster = tf.TransformBroadcaster()

        # Processing the sensor polling in an endless loop until this node goes to die
        while not rospy.is_shutdown():

            current_time = rospy.Time.now()
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

            # first, we'll publish the transform over tf
            transformBroadcaster.sendTransform(
                (.0, .0, .0),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(.0, .0, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(.0, .0, .0), Vector3(.0, .0, .0))

            # publish the message
            odomPub.publish(odom)

            rate.sleep()


if __name__ == '__main__':
    try:
        manager = DummyOdom()
        manager.start()
    except rospy.ROSInterruptException:
        pass
