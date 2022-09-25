#!/usr/bin/env python
import cv2
import rospy

from cv_bridge import CvBridge
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy
from sensor_msgs.msg import Image

DETECTOR_PROCESSOR_NODE = 'detector_processor_node'
CAMERA_IMAGE_RAW_TOPIC = '/camera/image_raw'

'''
How to launch

$roscore
$roslaunch video_stream_opencv camera.launch video_stream_provider:=/home/jetson/playground/videos/parking.avi visualize:=false loop_videofile:=true
$rosrun ros_image_subscriber detectnet_processor.py
'''

class DetectNetProcessor:

    bridge = CvBridge()
    net = detectNet('ssd-mobilenet-v2', threshold=0.5)

    def __init__(self):
        rospy.init_node(DETECTOR_PROCESSOR_NODE, anonymous=True)
        rospy.loginfo('%s started', DETECTOR_PROCESSOR_NODE)

    def subscribe_image(self):
        rospy.Subscriber(CAMERA_IMAGE_RAW_TOPIC, Image, self.__process_image)
        rospy.spin()

    def __process_image(self, image):
        try:
            origin = self.bridge.imgmsg_to_cv2(image, 'bgr8')

            rgb_img = cv2.cvtColor(origin, cv2.COLOR_BGR2RGB)
            
            cuda_mem = cudaFromNumpy(rgb_img)

            detections = self.net.Detect(cuda_mem, overlay='box,labels,conf')

            rospy.loginfo('Object Detection | Network %0f FPS | detected %d objects in image' % (self.net.GetNetworkFPS(), len(detections)))

            for detection in detections:
                left, top, right, bottom = int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom)
                color = (255, 0, 0)
                cv2.rectangle(origin, (left, top), (right, bottom), color)
                rospy.loginfo('%s, Confidence=%1f' % (self.net.GetClassDesc(detection.ClassID), detection.Confidence))

            cv2.imshow('Result', origin)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr('Exception %s' % (e.args))


if __name__ == '__main__':
    processor = DetectNetProcessor()
    processor.subscribe_image()