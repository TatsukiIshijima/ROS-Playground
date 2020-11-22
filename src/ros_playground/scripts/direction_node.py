#!/usr/bin/env python
import rospy

from ros_playground.msg import Infrared
from ros_playground.msg import Direction


INFRARED_TOPIC = "Infrared_Topic"

DIRECTION_TOPIC = "Direction_Topic"
DIRECTION_NODE = "Direction_Node"


class DirectionNode():
    def __init__(self):
        rospy.init_node(DIRECTION_NODE, anonymous=True)
        self.__direction_pub = rospy.Publisher(DIRECTION_TOPIC,
                                             Direction,
                                             queue_size=10)
    
    
    def subscribe_infrared(self):
        rospy.Subscriber(INFRARED_TOPIC, Infrared, self.__callback)
        rospy.spin()


    def __callback(self, infrared):
        log = ": right %s, center %s, left %s" % (infrared.right_value, infrared.center_value, infrared.left_value)
        rospy.loginfo(rospy.get_caller_id() + log)
        self.__publish_direction(infrared)

    def __publish_direction(self, infrared):
        infrared_list = [infrared.right_value,
                         infrared.center_value,
                         infrared.left_value]

        direction = Direction()
    
        # enable go to any direction
        if all([value <= 500 for value in infrared_list]):
            direction.value = 108
        # go to enable direction
        else:
            index = infrared_list.index(min(infrared_list))
            if index == 0:
                direction.value = 40
            elif index == 2:
                direction.value = 108
            else:
                direction.value = 74

        self.__direction_pub.publish(direction)


if __name__ == '__main__':
    direction_node = DirectionNode()
    direction_node.subscribe_infrared()
    
