#!/usr/bin/env python

from basestate import BaseState

class DoNothingState(BaseState):

    def __init__(self, pathmanager, successLambda, errorLambda):
        BaseState.__init__(self, pathmanager, successLambda, errorLambda)

        pathmanager.driver.stop()

        return