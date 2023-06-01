import rospy
import math
from basestate import BaseState
from pidcontroller import PIDController
from tf.transformations import euler_from_quaternion

from trigfunctions import signedangle
from rotatetostate import RotateToState

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

        def angleerror(current):
            dx = self.targetpose.pose.position.x - current.pose.position.x
            dy = self.targetpose.pose.position.y - current.pose.position.y

            odomQuat = robotcontroller.latestodominmapframe.pose.orientation
            (_, _, currentyaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

            targetyaw = math.atan2(dy, dx)

            return signedangle(currentyaw, targetyaw)

        if self.yawcontroller is None:
            self.yawcontroller = PIDController(0.025, .0, .0, angleerror)

        time = int(robotcontroller.latestodominmapframe.header.stamp.to_sec() * 1000)

        regvalyaw, yawerror = self.yawcontroller.update(time, robotcontroller.latestodominmapframe, None)
        regdistance, distanceerror = self.distanceController.update(time, robotcontroller.latestodominmapframe, None)

        if regvalyaw and regdistance:

            # In case there is a too large error in yaw, we just regulate the yaw, and not the velocity
            if abs(regvalyaw) > 0.3:
                robotcontroller.sendcontrol(.0, regvalyaw)
            else:
                robotcontroller.sendcontrol(regdistance, regvalyaw)

            if abs(regdistance) < 0.02:
                rospy.loginfo('DriveToPosition() - Finished distance = %s', regdistance)

                robotcontroller.sendcontrol(.0, .0)
                robotcontroller.finishbehavior()
