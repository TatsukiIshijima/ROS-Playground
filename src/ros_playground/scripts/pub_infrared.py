#!/usr/bin/env python
import rospy
import sys
import signal

from ros_playground.msg import Infrared
from PyMata.pymata import PyMata

INFRARED_ANALOG_PIN = 0

board = PyMata("/dev/ttyS0", verbose=True)
infrared_value = 0

def get_infrared_value():
    global infrared_value
    return infrared_value

def set_infrared_value(value):
    global infrared_value
    infrared_value = value

def signal_handler(sig, frame):
    if board is not None:
        board.reset()
    sys.exit(0)

def read_infrared_value(data):
    #print "infrared value :  %s" % (data[2])
    set_infrared_value(data[2])

def pub_infrared_value():
    signal.signal(signal.SIGINT, signal_handler)
    board.set_pin_mode(INFRARED_ANALOG_PIN, board.INPUT, board.ANALOG, read_infrared_value)
    pub = rospy.Publisher('chatter', Infrared, queue_size=10)
    rospy.init_node('infrared', anonymous=True)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        str = "Infrared value: %s" % (infrared_value)
        rospy.loginfo(str)
        infrared = Infrared()
        infrared.value = infrared_value
        pub.publish(infrared)
        r.sleep()

if __name__ == '__main__':
    try:
        pub_infrared_value()
    except rospy.ROSInterruptException:
        if board is not None:
            board.reset()

