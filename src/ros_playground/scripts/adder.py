#!/usr/bin/python

import rospy

from ros_playground.msg import Playground

def callback(data):
    print data.arg_x + data.arg_y

def adder():
    rospy.init_node('adder', anonymous=True)
    rospy.Subscriber('input_data', Playground, callback)
    rospy.spin()

if __name__ == '__main__':
    adder()
