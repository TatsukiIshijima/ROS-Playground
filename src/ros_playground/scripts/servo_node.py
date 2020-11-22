#!/usr/bin/env python
import rospy

from ros_playground.msg import Direction

DIRECTION_TOPIC = "Direction_Topic"

SERVO_NODE = "Servo_Node"


class ServoNode():
    def __init__(self):
        rospy.init_node(SERVO_NODE, anonymous=True)

    def subscribe_direction(self):
        rospy.Subscriber(DIRECTION_TOPIC, Direction, self.__callback)
        rospy.spin()

    def __callback(self, direction):
        log = ": direction %s" % (direction.value)
        rospy.loginfo(rospy.get_caller_id() + log)


if __name__ == '__main__':
    servo_node = ServoNode()
    servo_node.subscribe_direction()
