#!/usr/bin/env python

import rospy
from DeltaRobot import *
import time
import numpy as np

# from tensorrt_demos_ros.msg import *
from oakd_ros.msg import DetectionWithDepth



class ChestnutPicker:

	def __init__ (self):

		rospy.init_node("chestnut_picker_node", anonymous=True)
		print("Start Chestnut-Picker-ROS node")

		rospy.Subscriber("/oakd/detection", DetectionWithDepth, self.detection_callback)

		self.dr = DeltaRobot()
		self.dr.RobotTorqueOn()
		self.dr.GripperTorqueOn()
		self.dr.GoHome()
		self.dr.GripperCheck()


		### Detection's Parameters ###
		self.nboxes = 0
		self.xc = np.array([])
		self.yc = np.array([])
		self.closest_depth = np.array([])


		### Robot's Parameters ###
		self.Xlength = 840.0 #760.0 # This is a true length from camera view
		self.Ylength = 480.0 #575.0

		self.grabHeight = -763.0 #-745.0	#-745
		self.XBucket = 0.0
		self.YBucket = -280.0 #600.0
		self.ZBucket = -350.0 #-300.0

		self.XHome = 0.0
		self.YHome = 0.0
		self.ZHome = -329.9795

		self.finishTime = 1200

		self.cameraOffset = 85.0  #95.0
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
		self.closest_depth = np.array(msg.closest_depth.data)

		# print("xc", self.xc)
		# print("yc", self.yc)
		# print("nboxes", self.nboxes)



	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def pixelToCartesian(self, px, py):
		x = self.map(px,0,1280,-self.Xlength/2.0, self.Xlength/2.0) # use map() give similar result as coor_g()
		y = self.map(py,0,720,self.Ylength/2.0,-self.Ylength/2.0)
		return x,y


	def loop(self):

		rate = rospy.Rate(20)
		waitTime = 0.8
		grab_waitTime = 0.5
		goGome_waitTime = 1.2
		Z_est = self.grabHeight
		j = 0
		X = 200.0
		Y_with_offset = 200.0
		Z = -755.0

		while not rospy.is_shutdown():

			self.dr.GotoPoint(X,(Y_with_offset), Z)
			time.sleep(waitTime)
			self.dr.GripperClose()
			time.sleep(grab_waitTime)
			self.dr.GotoPoint(X,(Y_with_offset), self.ZBucket)
			time.sleep(waitTime)
			self.dr.GotoPoint(self.XBucket, 0.0,self.ZBucket)
			time.sleep(waitTime/2.0)
			self.dr.GotoPoint(self.XBucket,self.YBucket,self.ZBucket)
			time.sleep(waitTime)
			self.dr.GripperOpen()
			time.sleep(grab_waitTime)
			self.dr.GotoPoint(self.XBucket, 0.0,self.ZBucket)
			time.sleep(waitTime)
			self.dr.GoHome()
			time.sleep(goGome_waitTime)
	


			rate.sleep()

if __name__ == "__main__":

	CHP = ChestnutPicker()