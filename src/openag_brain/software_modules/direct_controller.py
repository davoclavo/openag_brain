#!/usr/bin/env python
"""
Python implementation an open-loop controller that just interprets the set point as a direct command to the actuator
"""
import rospy
from controllers import OpenLoopController

class Direct(OpenLoopController):
    def update(self, set_point):
        return set_point

if __name__ == '__main__':
    Direct.start()
