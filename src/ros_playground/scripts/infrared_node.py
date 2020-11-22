#!/usr/bin/env python
import rospy
import sys
import signal

from ros_playground.msg import Infrared
from PyMata.pymata import PyMata

INFRARED_TOPIC = "Infrared_Topic"
INFRARED_NODE = "Infrared_Node"

RIGHT_INFRARED_ANALOG_PIN = 0
LEFT_INFRARED_ANALOG_PIN = 1
CENTER_INFRARED_ANALOG_PIN = 2


class InfraredNode():
    def __init__(self):
        rospy.init_node(INFRARED_NODE, anonymous=True)
        self.board = PyMata("/dev/ttyS0", verbose=True)
        self.__infrared_pub = rospy.Publisher(INFRARED_TOPIC,
                                            Infrared,
                                            queue_size=10)

    def __signal_handler(self, sig, frame):
        if self.board is not None:
            self.board.reset()
        sys.exit(0)


    def publish_infrared(self):
        signal.signal(signal.SIGINT, self.__signal_handler)

        self.board.set_pin_mode(RIGHT_INFRARED_ANALOG_PIN, 
                                self.board.INPUT, 
                                self.board.ANALOG) 
        self.board.set_pin_mode(CENTER_INFRARED_ANALOG_PIN,
                                self.board.INPUT,
                                self.board.ANALOG)
        self.board.set_pin_mode(LEFT_INFRARED_ANALOG_PIN,
                                self.board.INPUT,
                                self.board.ANALOG)
    
        r = rospy.Rate(2)
    
        while not rospy.is_shutdown():
            right_value = self.board.analog_read(RIGHT_INFRARED_ANALOG_PIN)
            center_value = self.board.analog_read(CENTER_INFRARED_ANALOG_PIN)
            left_value = self. board.analog_read(LEFT_INFRARED_ANALOG_PIN)
            infrared = Infrared()
            infrared.right_value = right_value
            infrared.center_value = center_value
            infrared.left_value = left_value
            self.__infrared_pub.publish(infrared)
            r.sleep()


if __name__ == '__main__':
    infrared_node = InfraredNode()
    try:
        infrared_node.publish_infrared()
    except rospy.ROSInterruptException:
        if infrared_node.board is not None:
            infrared_node.board.reset()
