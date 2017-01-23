#!/usr/bin/python
"""
By convention, firmware modules publish sensor data to ROS topics in the
namespace `/sensors` and listen to actuator commands on ROS topics in the
namespace `/actuators`. This is very useful for low level tasks such as
debugging/testing your hardware but not so useful for getting a high level
overview of the environmental conditions of your system. For this, we would
like to use topics namespaced by the ID of the environment on which the piece
of hardware acts (e.g. /environment_1/measured/air_temperature). This module
connects topics so as to ensure that both of these system views work as
expected. There should be exactly one instance of this module in the system
"""
import sys
import time
import rospy
import rosgraph
import rostopic
from openag.cli.config import config as cli_config
from openag.utils import synthesize_firmware_module_info
from openag.models import FirmwareModule, FirmwareModuleType, SoftwareModule, SoftwareModuleType
from openag.db_names import FIRMWARE_MODULE, FIRMWARE_MODULE_TYPE, SOFTWARE_MODULE, SOFTWARE_MODULE_TYPE
from couchdb import Server
from std_msgs.msg import Float64, Bool

from openag_brain import params
from openag_brain.srv import Empty
from roslib.message import get_message_class

def connect_topics(
    src_topic, dest_topic, src_topic_type, dest_topic_type, multiplier=1,
    deadband=0
):
    rospy.loginfo("Connecting topic {} to topic {}".format(
        src_topic, dest_topic
    ))
    pub = rospy.Publisher(dest_topic, dest_topic_type, queue_size=10)
    def callback(src_item):
        val = src_item.data
        if src_topic_type == Float64:
            val *= multiplier
            if dest_topic_type == Bool:
                val = (val > deadband)
        dest_item = dest_topic_type(val)
        pub.publish(dest_item)
    sub = rospy.Subscriber(src_topic, src_topic_type, callback)

def connect_all_topics(server):
    firmware_module_db = server[FIRMWARE_MODULE]
    firmware_module_type_db = server[FIRMWARE_MODULE_TYPE]
    software_module_db = server[SOFTWARE_MODULE]
    software_module_type_db = server[SOFTWARE_MODULE_TYPE]

    firmware_modules = {
        firmware_module_id: FirmwareModule(firmware_module_db[firmware_module_id]) for firmware_module_id in
        firmware_module_db if not firmware_module_id.startswith('_')
    }
    firmware_module_types = {
        type_id: FirmwareModuleType(firmware_module_type_db[type_id]) for type_id in
        firmware_module_type_db if not type_id.startswith("_")
    }
    software_modules = {
        software_module_id: SoftwareModule(software_module_db[software_module_id]) for software_module_id in
        software_module_db if not software_module_id.startswith('_')
    }
    software_module_types = {
        type_id: SoftwareModuleType(software_module_type_db[type_id]) for type_id in
        software_module_type_db if not type_id.startswith("_")
    }
    firmware_modules = synthesize_firmware_module_info(firmware_modules, firmware_module_types)
    for firmware_module_id, firmware_module_info in firmware_modules.items():
        for input_name, input_info in firmware_module_info["inputs"].items():
            if not "actuators" in input_info["categories"]:
                continue
            src_topic = "/environments/{}/{}/commanded".format(
                firmware_module_info["environment"], input_info["variable"]
            )
            dest_topic = "/actuators/{}/{}".format(firmware_module_id, input_name)

            # Extract controller command topic type from software module type definitions
            controller_module = next(software_module for software_module_id, software_module in software_modules.items() if software_module.get("parameters", {}).get("variable") == input_info["variable"])
            controller_module_type = software_module_types[controller_module["type"]]
            src_topic_type = get_message_class(controller_module_type["outputs"]["cmd"]["type"])

            dest_topic_type = get_message_class(input_info["type"])
            connect_topics(
                src_topic, dest_topic, src_topic_type, dest_topic_type,
                multiplier=input_info.get("multiplier", 1),
                deadband=input_info.get("deadband", 0)
            )
        for output_name, output_info in firmware_module_info["outputs"].items():
            if not "sensors" in output_info["categories"]:
                continue
            src_topic = "/sensors/{}/{}/raw".format(firmware_module_id, output_name)
            dest_topic = "/environments/{}/{}/raw".format(
                firmware_module_info["environment"], output_info["variable"]
            )
            src_topic_type = get_message_class(output_info["type"])
            dest_topic_type = Float64
            connect_topics(
                src_topic, dest_topic, src_topic_type, dest_topic_type
            )

if __name__ == '__main__':
    rospy.init_node("topic_connector")
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local server specified")
    server = Server(db_server)
    connect_all_topics(server)
    rospy.spin()
