#!/usr/bin/python

import rospy

from ros_playground.msg import Playground

def para_in():

    rospy.init_node('para_in', anonymous=True)

    pub = rospy.Publisher('input_data', Playground, queue_size=100)

    r = rospy.Rate(5)

    para_x = 0
    para_y = 2

    msg = Playground()

    while not rospy.is_shutdown():
        msg.arg_x = para_x
        msg.arg_y = para_y

        pub.publish(msg)
        print "published arg_x=%d arg_y=%d"%(msg.arg_x,msg.arg_y)
        para_x += 1
        para_y += 1

        r.sleep()

if __name__ == '__main__':
    try:
         para_in()
    except rospy.ROSInterruptException: 
        pass
