#!/usr/bin/env python
"""
Python implementation of an direct controller, also known as proportional controller.

  multiplier[Float]: A scaling value for the proportinal gain. Useful for things that have a negative effort on the state (e.g. a Cooler)
"""
import rospy
from controllers import Controller
from std_msgs.msg import Float32

class Direct(Controller):
    output_type = Float32

    def __init__(self, multiplier=1.0):
        self.multiplier = multiplier

        self.set_point = None
        self.last_error = 0

    def update(self, state):
        if self.set_point is None:
            return

        error = self.set_point - state
        res = error * self.multiplier
        return res

if __name__ == '__main__':
    Direct.start()
