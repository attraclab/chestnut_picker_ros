#!/usr/bin/env python

import rospy
from DeltaRobot import *
import time
import numpy as np

from tensorrt_demos_ros.msg import *


class ChestnutPicker:

	def __init__ (self):

		rospy.init_node("chestnut_picker_node", anonymous=True)
		print("Start Chestnut-Picker-ROS node")

		rospy.Subscriber("/detection", Detection, self.detection_callback)

		self.dr = DeltaRobot()
		self.dr.RobotTorqueOn()
		self.dr.GripperTorqueOn()
		self.dr.GoHome()
		self.dr.GripperCheck()


		### Detection's Parameters ###
		self.nboxes = 0
		self.xc = np.array([])
		self.yc = np.array([])


		### Robot's Parameters ###
		self.Xlength = 760.0 # This is a true length from camera view
		self.Ylength = 575.0

		self.grabHeight = -745.0	#-745
		self.XBucket = 0.0
		self.YBucket = 600.0
		self.ZBucket = -300.0

		self.XHome = 0.0
		self.YHome = 0.0
		self.ZHome = -329.9795

		self.finishTime = 1200

		self.cameraOffset = 80.0  #95.0
		self.roverOffset = 55.0   #on carpet 90.0 with 5rpm

		### Motion Parameters ###
		self.goPick = False
		self.BUSY = False
		self.moveFinished = False
		self.grabFinished = False


		self.loop()

		rospy.spin()

	def detection_callback(self, msg):

		self.nboxes = msg.nboxes

		self.xc = np.array(msg.xc.data)
		self.yc = np.array(msg.yc.data)

		# print("xc", self.xc)
		# print("yc", self.yc)
		# print("nboxes", self.nboxes)



	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def pixelToCartesian(self, px, py):
		x = self.map(px,0,640,-self.Xlength/2.0, self.Xlength/2.0) # use map() give similar result as coor_g()
		y = self.map(py,0,480,self.Ylength/2.0,-self.Ylength/2.0)
		return x,y


	def loop(self):

		rate = rospy.Rate(20)
		waitTime = 0.8
		grab_waitTime = 0.5
		goGome_waitTime = 1.2

		j = 0
		while not rospy.is_shutdown():

			if self.nboxes > 0:

				X,Y = self.pixelToCartesian(self.xc[0], self.yc[0])
				Y_with_offset = Y+self.cameraOffset
				print("X: {:.2f} | Y: {:.2f} | Y cam off: {:.2f}".format(\
					X,Y,Y_with_offset))

				if (Y_with_offset < 370.0):
					self.dr.GotoPoint(X,(Y_with_offset), self.grabHeight)
					time.sleep(waitTime)
					self.dr.GripperClose()
					time.sleep(grab_waitTime)
					self.dr.GotoPoint(X,(Y_with_offset), self.ZBucket)
					time.sleep(waitTime)
					self.dr.GotoPoint(self.XBucket,self.YBucket-200,self.ZBucket)
					time.sleep(waitTime/2.0)
					self.dr.GotoPoint(self.XBucket,self.YBucket,self.ZBucket)
					time.sleep(waitTime)
					self.dr.GripperOpen()
					time.sleep(grab_waitTime)
					self.dr.GotoPoint(self.XBucket,self.YBucket-200,self.ZBucket)
					time.sleep(waitTime)
					self.dr.GoHome()
					time.sleep(goGome_waitTime)
				else:
					print("Too close to front frame")

			rate.sleep()

if __name__ == "__main__":

	CHP = ChestnutPicker()