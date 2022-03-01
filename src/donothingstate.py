#!/usr/bin/env python3

import rospy

from basestate import BaseState


class DoNothingState(BaseState):

    def __init__(self, pathmanager, successLambda, errorLambda):
        BaseState.__init__(self, pathmanager, successLambda, errorLambda)

        rospy.loginfo("Current State : DoNothingState")
        pathmanager.driver.stop()

    def process(self):
        self.pathmanager.publishNavigationInfo(.0, .0)
        return self
