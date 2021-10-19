#!/usr/bin/env python

class BaseState:

    def __init__(self, pathmanager):
        self.pathmanager = pathmanager
        return

    def process(self):
        return self

    def abort(self):
        return