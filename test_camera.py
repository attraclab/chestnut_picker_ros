#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def loop():
	image_pub = rospy.Publisher("/webcam", Image, queue_size=1)
	bridge = CvBridge()

	rate = rospy.Rate(50)

	frame_width = 640
	frame_height = 480
	video = cv2.VideoCapture("/dev/video0")
	video.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
	video.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

	while not rospy.is_shutdown():

		_, frame = video.read()

		try:
			image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8") #bgr8 #8UC1
			# bw_image_message = self.bridge.cv2_to_imgmsg(processed_img, encoding="8UC1")
			image_pub.publish(image_message)
		except CvBridgeError as e:
			print(e)

		cv2.waitKey(1)

		rate.sleep()






if __name__ == "__main__":
	
	rospy.init_node("test_camera_node", anonymous=True)
	loop()

	rospy.spin()