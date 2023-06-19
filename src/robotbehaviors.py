import rospy
import math

from pidcontroller import PIDController
from tf.transformations import euler_from_quaternion

from trigfunctions import signedangle

class BaseState:

    def __init__(self):
        pass

    def process(self, controller):
        pass

class RotateToState(BaseState):

    def __init__(self, targettheta):

        BaseState.__init__(self)

        def errorfun(current):
            odomquat = current.pose.orientation
            (_, _, yaw) = euler_from_quaternion([odomquat.x, odomquat.y, odomquat.z, odomquat.w])

            return signedangle(yaw, targettheta)

        self.controller = PIDController(0.025, .0, .0, errorfun)

    def process(self, robotcontroller):

        regval, error = self.controller.update(int(robotcontroller.latestodominmapframe.header.stamp.to_sec() * 1000), robotcontroller.latestodominmapframe)
        if regval:
            if abs(regval) < 0.01:
                rospy.loginfo('RotateToState() - Finished')

                robotcontroller.sendcontrol(.0, .0)
                robotcontroller.finishbehavior()
            else:
                robotcontroller.sendcontrol(.0, regval)

class DriveToPosition(BaseState):

    def __init__(self, targetpose):

        BaseState.__init__(self)

        self.targetpose = targetpose

        def distanceerrorfun(current):

            dx = self.targetpose.pose.position.x - current.pose.position.x
            dy = self.targetpose.pose.position.y - current.pose.position.y

            return math.sqrt(dx * dx + dy * dy)

        self.distanceController = PIDController(1.2, .0, .0, distanceerrorfun)
        self.yawcontroller = None

    def process(self, robotcontroller):

        def angleerror(current):
            dx = self.targetpose.pose.position.x - current.pose.position.x
            dy = self.targetpose.pose.position.y - current.pose.position.y

            odomquat = robotcontroller.latestodominmapframe.pose.orientation
            (_, _, currentyaw) = euler_from_quaternion([odomquat.x, odomquat.y, odomquat.z, odomquat.w])

            targetyaw = math.atan2(dy, dx)

            return signedangle(currentyaw, targetyaw)

        if self.yawcontroller is None:
            self.yawcontroller = PIDController(0.025, .0, .0, angleerror)

        time = int(robotcontroller.latestodominmapframe.header.stamp.to_sec() * 1000)

        regvalyaw, yawerror = self.yawcontroller.update(time, robotcontroller.latestodominmapframe)
        regdistance, distanceerror = self.distanceController.update(time, robotcontroller.latestodominmapframe)

        if regvalyaw and regdistance:

            # In case there is a too large error in yaw, we just regulate the yaw, and not the velocity
            if abs(yawerror) > 5:
                robotcontroller.sendcontrol(.0, regvalyaw)
            else:
                robotcontroller.sendcontrol(regdistance, regvalyaw)

            if abs(regdistance) < 0.02:
                rospy.loginfo('DriveToPosition() - Finished distance = %s', regdistance)

                robotcontroller.sendcontrol(.0, .0)
                robotcontroller.finishbehavior()
