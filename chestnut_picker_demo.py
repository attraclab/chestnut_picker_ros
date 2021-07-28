#!/usr/bin/env python

import rospy
from DeltaRobot import *
import time
import numpy as np

from tensorrt_demos_ros.msg import *
from std_msgs.msg import Int32MultiArray


class ChestnutPicker:

	def __init__ (self):

		rospy.init_node("chestnut_picker_node", anonymous=True)
		print("Start Chestnut-Picker-ROS node")

		rospy.Subscriber("/detection", Detection, self.detection_callback)
		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_ch_callback)

		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		self.dr = DeltaRobot()
		self.dr.RobotTorqueOn()
		self.dr.GripperTorqueOn()
		self.dr.GoHome()
		self.dr.GripperCheck()


		### Detection's Parameters ###
		self.nboxes = 0
		self.xc = np.array([])
		self.yc = np.array([])


		### Robot Arm's Parameters ###
		## TODO : change Xlenght, Ylength according to how much the robot in home position can see in that frame size
		##        change YBucket, ZBucket according to the height and distance of bucket
		##        change grabHeight to the height where robot can grab easily
		## use DR_Kinematics.py script to get robot coordinates of each position
		self.Xlength = 760.0 # This is a true length from camera view
		self.Ylength = 575.0

		self.grabHeight = -745.0	#-720.0 #-725.0  #-723.0
		self.XBucket = 0.0
		self.YBucket = 600.0
		self.ZBucket = -300.0

		self.XHome = 0.0
		self.YHome = 0.0
		self.ZHome = -329.9795

		self.finishTime = 1200

		self.cameraOffset = 80.0  #95.0
		self.roverOffset = 45.0   #according to the throttle speed

		self.ROBOT_MODE = "MANUAL"
		self.picker_flag = True

		### Motion Parameters ###
		self.waitTime = 0.8
		self.grab_waitTime = 0.5
		self.goGome_waitTime = 1.2

		self.const_throttle = 1120	#1140
		self.const_bwd_throttle = 960	#870


		self.loop()

		rospy.spin()

	def detection_callback(self, msg):

		self.nboxes = msg.nboxes

		self.xc = np.array(msg.xc.data)
		self.yc = np.array(msg.yc.data)

		# print("xc", self.xc)
		# print("yc", self.yc)
		# print("nboxes", self.nboxes)

	def sbus_ch_callback(self, msg):

		if msg.data[4] < 700:
			self.ROBOT_MODE = "HOLD"
		elif msg.data[4] < 1500:
			self.ROBOT_MODE = "MANUAL"
		else:
			self.ROBOT_MODE = "AUTO"

		## ch6 [5] for pwmcart
		## ch7 [6] for atcart
		if msg.data[6] > 1500:
			self.picker_flag = True
		else:
			self.picker_flag = False





	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def pixelToCartesian(self, px, py):
		x = self.map(px,0,640,-self.Xlength/2.0, self.Xlength/2.0) # use map() give similar result as coor_g()
		y = self.map(py,0,480,self.Ylength/2.0,-self.Ylength/2.0)
		return x,y

	def go_pick(self, from_move_flag):

		## check it again here, if object is still there
		if self.nboxes > 0:
			X,Y = self.pixelToCartesian(self.xc[0], self.yc[0])
		
			## from_move_flag is telling that the robot has initial speed or not
			## if it has then we add some rover_offset 
			if from_move_flag:
				Y_with_offset = Y+self.cameraOffset+self.roverOffset
			else:
				Y_with_offset = Y+self.cameraOffset

			## check if object is close to left/right frame
			## we modified X to narrower, but it could still pick
			if X > 310.0:
				print("old X", X)
				X = 310.0
			elif X < -295.0:
				print("old X", X)
				X = -295.0

			print("X: {:.2f} | Y: {:.2f} | Y cam off: {:.2f} | rover_offset: {:}".format(\
				X,Y,Y_with_offset, from_move_flag))

			## if Y is in the proper zone
			if (Y_with_offset < 350.0):
				self.dr.GotoPoint(X,(Y_with_offset), self.grabHeight)
				time.sleep(self.waitTime)
				self.dr.GripperClose()
				time.sleep(self.grab_waitTime)
				self.dr.GotoPoint(X,(Y_with_offset), self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GotoPoint(self.XBucket,self.YBucket-200,self.ZBucket)
				time.sleep(self.waitTime/2.0)
				self.dr.GotoPoint(self.XBucket,self.YBucket,self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GripperOpen()
				time.sleep(self.grab_waitTime)
				self.dr.GotoPoint(self.XBucket,self.YBucket-200,self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GoHome()
				time.sleep(self.goGome_waitTime)
			else:
				print("Too close to front frame, go back a bit...")
				sbus_throttle = self.const_bwd_throttle
				sbus_steering = 1024
				self.sbus_cmd.data = [sbus_steering, sbus_throttle]
				self.sbus_cmd_pub.publish(self.sbus_cmd)
				time.sleep(1.0)



	def loop(self):

		rate = rospy.Rate(20)
		from_pick_flag = False
	
		j = 0
		while not rospy.is_shutdown():

			if (self.nboxes > 0) and (self.picker_flag):

				sbus_throttle = 1024
				sbus_steering = 1024
				self.sbus_cmd.data = [sbus_steering, sbus_throttle]
				self.sbus_cmd_pub.publish(self.sbus_cmd)

				self.go_pick(from_move_flag)


				## Wait a bit after stop
				# if from_move_flag and self.ROBOT_MODE == "AUTO":
				# 	print("wait from move")
				# 	time.sleep(1.5)


				from_move_flag = False
				from_pick_flag = True

			else:

				if from_pick_flag:
					time.sleep(0.5)

				sbus_throttle = self.const_throttle
				sbus_steering = 1024
				from_move_flag = True
				from_pick_flag = False

				self.sbus_cmd.data = [sbus_steering, sbus_throttle]

				self.sbus_cmd_pub.publish(self.sbus_cmd)


			rate.sleep()

if __name__ == "__main__":

	CHP = ChestnutPicker()