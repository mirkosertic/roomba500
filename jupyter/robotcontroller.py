import rospy
import threading
import tf

from idlestate import IdleState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class RobotController:
    
    def __init__(self, transformlistener, mapframe, cmdvelpub):
        self.lock = threading.Lock()
        self.transformlistener = transformlistener
        self.mapframe = mapframe
        self.behavior = []
        self.latestodominmapframe = None
        self.cmdvelpub = cmdvelpub
        
    def cap(self, value, m):
        if value > m:
            value = m
        if value < -m:
            value = -m
        return value

    def sendcontrol(self, velx, veltheta):
        twistMsg = Twist()
        twistMsg.linear.x = self.cap(velx, 0.4)
        twistMsg.linear.y = .0
        twistMsg.linear.z = .0
        twistMsg.angular.x = .0
        twistMsg.angular.y = .0
        twistMsg.angular.z = self.cap(veltheta, 3.0)
        self.cmdvelpub.publish(twistMsg)
        
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
            rospy.logerr('Error processing odometry data : %s', e)
            
        self.lock.release()            
        pass
    
    def processBehavior(self):
        try:
            if len(self.behavior) > 0:
                self.behavior[-1].process(self)
        except Exception as e:                
            rospy.logerr('Error processing behavior : %s', e)
        pass
    
    def appendbehavior(self, b):
        self.lock.acquire()
        self.behavior.append(b)
        
        self.lock.release()
        
    def finishbehavior(self):
        del self.behavior[-1]