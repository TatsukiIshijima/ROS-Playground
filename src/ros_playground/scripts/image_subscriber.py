#!/usr/bin/env python
import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

IMAGE_SUBSCRIBER_NODE = 'image_subscriber_node'
IMAGE_SUBSCRIBER_TOPIC = '/camera/image_raw'

'''
How to launch

$roscore
$roslaunch video_stream_opencv camera.launch video_stream_provider:=/home/jetson/playground/videos/parking.avi visualize:=false loop_videofile:=true
$rosrun ros_image_subscriber image_subscriber.py
'''

class ImageSubscriber:

    bridge = CvBridge()

    def __init__(self):
        rospy.init_node(IMAGE_SUBSCRIBER_NODE, anonymous=True)
        rospy.loginfo('%s started', IMAGE_SUBSCRIBER_NODE)

    def subscribe_image(self):
        rospy.Subscriber(IMAGE_SUBSCRIBER_TOPIC, Image, self.__process_image)
        rospy.spin()

    def __process_image(self, image):
        try:
            rospy.loginfo('width=%d, height=%d' % (image.width, image.height))
            origin = self.bridge.imgmsg_to_cv2(image, 'bgr8')

            cv2.imshow('image', origin)
            cv2.waitKey(1)
        except Exception as e:
            print('Exception %s', e.args)

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.subscribe_image()