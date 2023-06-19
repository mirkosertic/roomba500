import rospy
import threading
import tf
import traceback

from geometry_msgs.msg import PoseStamped

class RobotController:

    def __init__(self, transformlistener, mapframe, driver):
        self.lock = threading.Lock()
        self.transformlistener = transformlistener
        self.mapframe = mapframe
        self.behavior = []
        self.latestodominmapframe = None
        self.driver = driver

    def cap(self, value, m):
        if value > m:
            value = m
        if value < -m:
            value = -m
        return value

    def sendcontrol(self, velx, veltheta):
        self.driver.drive(self.cap(velx, 0.4), self.cap(veltheta, 3.0))

    def stop(self):
        try:
            self.lock.acquire()
            
            self.behavior = []
            self.sendcontrol(.0, .0)
            
        except Exception as e:
            rospy.logerr('Error stopping action : %s', e)

        self.lock.release()
        pass
      
        
    def newodometry(self, message):
        try:
            self.lock.acquire()

            latestcommontime = self.transformlistener.getLatestCommonTime(self.mapframe, message.header.frame_id)

            mpose = PoseStamped()
            mpose.pose.position = message.pose.pose.position
            mpose.pose.orientation = message.pose.pose.orientation
            mpose.header.frame_id = message.header.frame_id
            mpose.header.stamp = latestcommontime

            self.latestodominmapframe = self.transformlistener.transformPose(self.mapframe, mpose)

            self.processBehavior()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('Error processing odometry data : %s', traceback.format_exc())

        self.lock.release()
        pass

    def processBehavior(self):
        try:
            if len(self.behavior) > 0:
                self.behavior[-1].process(self)
        except Exception as e:
            rospy.loginfo('Error processing behavior : %s', traceback.format_exc())
        pass

    def appendbehavior(self, b):
        self.lock.acquire()
        self.behavior.append(b)

        self.lock.release()

    def finishbehavior(self):
        del self.behavior[-1]
