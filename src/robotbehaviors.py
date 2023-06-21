import rospy
import math

from pidcontroller import PIDController
from tf.transformations import euler_from_quaternion

from trigfunctions import signedangle

class BaseState:

    def __init__(self):
        self.firstcalled = True
        pass

    def process(self, controller):
        pass

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

    def __str__(self):
        return "(DriveToPosition x=" + str(self.targetpose.pose.position.x) + " y=" + str(self.targetpose.pose.position.y) + ")"

    def __repr__(self):
        return self.__str__()

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

        if self.firstcalled:
            self.firstcalled = False
            rospy.loginfo("DriveToPosition() starting to reach %s:%s, distanceerror=%s, angleerror=%s", self.targetpose.pose.position.x, self.targetpose.pose.position.y, distanceerror, yawerror)

        if regvalyaw and regdistance:

            # In case there is a too large error in yaw, we just regulate the yaw, and not the velocity
            if abs(regdistance) < 0.025:
                rospy.loginfo('DriveToPosition() - Finished distance = %s', regdistance)

                robotcontroller.sendcontrol(.0, .0)
                robotcontroller.finish_behavior()

            elif abs(yawerror) > 3 and regdistance > 0.10:
                robotcontroller.sendcontrol(.0, regvalyaw)

            else:
                robotcontroller.sendcontrol(regdistance, regvalyaw)
