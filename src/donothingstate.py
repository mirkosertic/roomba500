#!/usr/bin/env python

from basestate import BaseState

class DoNothingState(BaseState):

    def __init__(self, pathmanager):
        BaseState.__init__(self, pathmanager)

        pathmanager.driver.stop()

        return