#!/usr/bin/env python

import rospy
import socket
import pickle
from oakd_ros.msg import DetectionWithDepth


class DataConverter:

	def __init__(self):

		rospy.init_node("data_conter_node", anonymous=True)

		PORT = 8888
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(("0.0.0.0", PORT))
		self.sock.setblocking(0)

		self.detect_pub = rospy.Publisher("/oakd/detection", DetectionWithDepth, queue_size=10)
		self.detect_msg = DetectionWithDepth()

		self.loop()
		rospy.spin()


	def loop(self):

		rate = rospy.Rate(50)

		print("start converting")
		while not rospy.is_shutdown():

			try:
				data, addr = self.sock.recvfrom(30000)
				# print("data len", len(data))
				parse_data = pickle.loads(data)
				
				print("nboxes ", parse_data['nboxes'])
				print("xc", parse_data['xc'])
				print("yc", parse_data['yc'])
				print("depth", parse_data['closest_depth'])

				for i in range(parse_data['nboxes']):
					self.detect_msg.xc.data.append(int(parse_data['xc'][i]))
					self.detect_msg.yc.data.append(int(parse_data['yc'][i]))
					self.detect_msg.closest_depth.data.append(parse_data['closest_depth'][i])

				self.detect_msg.nboxes = int(parse_data['nboxes'])
				

				self.detect_pub.publish(self.detect_msg)

				self.detect_msg = DetectionWithDepth()
			except socket.error:
				pass


			rate.sleep()

if __name__ == "__main__":

	a = DataConverter()