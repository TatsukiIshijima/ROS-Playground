#!/usr/bin/env python
import rospy
import sys
import signal

from ros_playground.msg import Infrared
from PyMata.pymata import PyMata

RIGHT_INFRARED_ANALOG_PIN = 0
LEFT_INFRARED_ANALOG_PIN = 1
CENTER_INFRARED_ANALOG_PIN = 2

board = PyMata("/dev/ttyS0", verbose=True)


def signal_handler(sig, frame):
    if board is not None:
        board.reset()
    sys.exit(0)


def pub_infrared_value():
    signal.signal(signal.SIGINT, signal_handler)

    board.set_pin_mode(RIGHT_INFRARED_ANALOG_PIN, 
                       board.INPUT, 
                       board.ANALOG) 
    board.set_pin_mode(CENTER_INFRARED_ANALOG_PIN,
                       board.INPUT,
                       board.ANALOG)
    board.set_pin_mode(LEFT_INFRARED_ANALOG_PIN,
                       board.INPUT,
                       board.ANALOG)
    
    pub = rospy.Publisher('chatter', Infrared, queue_size=10)
    
    rospy.init_node('infrared', anonymous=True)
    r = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        right_value = board.analog_read(RIGHT_INFRARED_ANALOG_PIN)
        center_value = board.analog_read(CENTER_INFRARED_ANALOG_PIN)
        left_value = board.analog_read(LEFT_INFRARED_ANALOG_PIN)
        infrared = Infrared()
        infrared.right_value = right_value
        infrared.center_value = center_value
        infrared.left_value = left_value
        str = "Infrared right: %s, center: %s, left: %s" % (infrared.right_value, infrared.center_value, infrared.left_value) 
        rospy.loginfo(str)
        pub.publish(infrared)
        r.sleep()

if __name__ == '__main__':
    try:
        pub_infrared_value()
    except rospy.ROSInterruptException:
        if board is not None:
            board.reset()

