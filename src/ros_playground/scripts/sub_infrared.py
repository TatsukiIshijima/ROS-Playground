#!/usr/bin/env python
import rospy

from ros_playground.msg import Infrared
from ros_playground.msg import Direction


INFRARED_TOPIC = "Infrared"
INFRARED_SUB_NODE = "Infrared_Sub_Node"

DIRECTION_TOPIC = "Direction"
DIRECTION_PUB_NODE = "Direction_Pub_Node"


rospy.init_node(DIRECTION_PUB_NODE, anonymous=True)
direction_pub = rospy.Publisher(DIRECTION_TOPIC, 
                                Direction, 
                                queue_size=10)


def callback(infrared):
    log = ": right %s, center %s, left %s" % (infrared.right_value, infrared.center_value, infrared.left_value)
    rospy.loginfo(rospy.get_caller_id() + log)
    
    pub_direction_value(infrared)


def pub_direction_value(infrared):
    infrared_list = [infrared.right_value,
                     infrared.center_value,
                     infrared.left_value]
    
    direction = Direction()
    
    # 全方向が開けている場合は中央
    if all([value <= 500 for value in infrared_list]):
        direction.value = 108
    # 開いている方向へ向かせる
    else:
        index = infrared_list.index(min(infrared_list))
        if index == 0:
            direction.value = 40
        elif index == 2:
            direction.value = 108
        else:
            direction.value = 74

    direction_pub.publish(direction)


def sub_infrared_value():
    rospy.init_node(INFRARED_SUB_NODE, anonymous=True)
    rospy.Subscriber(INFRARED_TOPIC, Infrared, callback)
    rospy.spin()


if __name__ == '__main__':
    sub_infrared_value()
    
