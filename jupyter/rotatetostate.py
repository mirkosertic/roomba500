import rospy

from basestate import BaseState
from pidcontroller import PIDController
from tf.transformations import euler_from_quaternion

from trigfunctions import signedangle

class RotateToState(BaseState):

    def __init__(self, targettheta):

        def errorfun(current):
            odomQuat = current.pose.orientation
            (_, _, yaw) = euler_from_quaternion([odomQuat.x, odomQuat.y, odomQuat.z, odomQuat.w])

            return signedangle(yaw, targettheta)

        self.controller = PIDController(0.025, .0, .0, errorfun)
        pass

    def process(self, robotcontroller):

        regval, error = self.controller.update(int(robotcontroller.latestodominmapframe.header.stamp.to_sec() * 1000), robotcontroller.latestodominmapframe, lambda value : robotcontroller.sendcontrol(.0, value))
        if regval and abs(regval) < 0.01:
            rospy.loginfo('RotateToState() - Finished')

            robotcontroller.sendcontrol(.0, .0)
            robotcontroller.finishbehavior()


