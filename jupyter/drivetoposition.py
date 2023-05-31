import rospy
import math
from basestate import BaseState
from pidcontroller import PIDController
from tf.transformations import euler_from_quaternion

from trigfunctions import signedangle

class DriveToPosition(BaseState):

    def __init__(self, targetpose):
        self.targetpose = targetpose

        def distanceerrorfun(current):

            dx = self.targetpose.pose.position.x - current.pose.position.x
            dy = self.targetpose.pose.position.y - current.pose.position.y

            return math.sqrt(dx * dx + dy * dy)

        self.distanceController = PIDController(1.2, .0, .0, distanceerrorfun)
        self.yawcontroller = None
        pass

    def process(self, robotcontroller):

        if self.yawcontroller is None:

            def angleerror(current):
                dx = self.targetpose.pose.position.x - current.pose.position.x
                dy = self.targetpose.pose.position.y - current.pose.position.y

                odomQuat = robotcontroller.latestodominmapframe.pose.orientation
                (_, _, currentyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

                targetyaw = math.atan2(dy, dx)

                return signedangle(currentyaw, targetyaw)

            self.yawcontroller = PIDController(0.05, .0, .0, angleerror)

        time = int(robotcontroller.latestodominmapframe.header.stamp.to_sec() * 1000)

        regvalyaw = self.yawcontroller.update(time, robotcontroller.latestodominmapframe, None)
        regdistance = self.distanceController.update(time, robotcontroller.latestodominmapframe, None)

        if regvalyaw and regdistance:

            robotcontroller.sendcontrol(regdistance, regvalyaw)

            if abs(regdistance) < 0.02:
                rospy.loginfo('DriveToPosition() - Finished distance = %s', regdistance)

                robotcontroller.sendcontrol(.0, .0)
                robotcontroller.finishbehavior()
