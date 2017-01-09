#!/usr/bin/env python
"""
Base class inherited to define a closed loop control module.

If arguments are defined in the `__init__` method, they will be
loaded automatically from the private ROS params defined in the
launchfile.

It subscribes to the setpoint and measurmeent ROS topics.

    Set point  - /<variable>/desired
    State      - /<variable>/measured

Every time a measurement is received by the controller,
it will execute the update function that returns a new
commanded output that is published to the following ROS topic:

    Output     - /<variable>/commanded

NOTE:
The `output_type` class attr has to be set if its other than Float32!
Only when new measurements are received a new output is published,
not when a new desired point is sent.

See direct_controller.py for a simple implementation of a controller
"""
import rospy
from re import sub
from inspect import getargspec
from std_msgs.msg import Float64, Float32

class Controller(object):
    set_point = None
    output_type = Float32

    def update(self, state):
        raise NotImplementedError(
            "Must `def update(self, state):` to use this base class"
        )

    @classmethod
    def start(cls):
        node_name = to_snake_case(cls.__name__)
        rospy.init_node(node_name)
        # Make sure that we're under an environment namespace.
        namespace = rospy.get_namespace()
        if namespace == '/':
            raise RuntimeError(
                "Cannot be run in the global namespace. Please "
                "designate an environment for this module."
            )

        # Get args of the initializer.
        # The first arg is the instance, so just extract the rest
        # Note: *args or **kwargs won't be used
        param_values = {}
        try:
            param_names = getargspec(cls.__init__).args[1:]
            for param_name in param_names:
                private_param_name = "~" + param_name
                if rospy.has_param(private_param_name):
                    param_values[param_name] = rospy.get_param(private_param_name)
        except TypeError:
            # raised by getargspec if no __init__ is defined,
            # which means there are no custom init params
            pass

        controller = cls(**param_values)

        # If variable is not present, this raises an exception
        variable = rospy.get_param("~variable")

        pub_name = "{}/commanded".format(variable)
        state_sub_name = "{}/measured".format(variable)
        desired_sub_name = "{}/desired".format(variable)

        rospy.logdebug("Starting {} controller for {}".format(node_name, variable))
        pub = rospy.Publisher(pub_name, cls.output_type, queue_size=10)

        def state_callback(item):
            rospy.logdebug("New {} measurement: {}".format(variable, item.data))
            cmd = controller.update(item.data)
            if cmd is not None:
                rospy.logdebug("New {} output: {}".format(variable, cmd))
                pub.publish(cmd)

        def set_point_callback(item):
            rospy.logdebug("New {} set point: {}".format(variable, item.data))
            controller.set_point = item.data

        state_sub = rospy.Subscriber(state_sub_name, Float64, state_callback)
        set_point_sub = rospy.Subscriber(
            desired_sub_name, Float64, set_point_callback
        )

        rospy.spin()

# Convert CamelCase to snake_case
# From http://stackoverflow.com/a/1176023/756000
def to_snake_case(name):
    s1 = sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
