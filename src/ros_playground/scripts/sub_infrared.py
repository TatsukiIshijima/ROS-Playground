#!/usr/bin/env python
import rospy

from ros_playground.msg import Infrared

INFRARED_TOPIC = "Infrared"
INFRARED_SUB_NODE = "Infrared_Sub_Node"


def callback(infrared):
    log = ": right %s, center %s, left %s" % (infrared.right_value, infrared.center_value, infrared.left_value)
    rospy.loginfo(rospy.get_caller_id() + log)


def sub_infrared_value():
    rospy.init_node(INFRARED_SUB_NODE, anonymous=True)
    rospy.Subscriber(INFRARED_TOPIC, Infrared, callback)
    rospy.spin()


if __name__ == '__main__':
    sub_infrared_value()
    
