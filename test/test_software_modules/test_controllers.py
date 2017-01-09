from nose.tools import assert_equal
from openag_brain.software_modules.controllers import Controller
from std_msgs.msg import Float64

class EmptyController(Controller):
    pass

class BasicController(Controller):
    output_type = Float64
    def update(self, state):
        return state

BasicController.start()
