#!/usr/bin/env python

import rospy
import rospkg
import rosparam
import yaml
from DeltaRobot import *
import time
import numpy as np
import os
import argparse
# from tensorrt_demos_ros.msg import *
from oakd_ros.msg import DetectionWithDepth
from std_msgs.msg import Int16MultiArray, UInt8
from geometry_msgs.msg import Twist


class ChestnutPicker(object):

	def __init__ (self, params_file):

		rospy.init_node("chestnut_picker_node", anonymous=True)
		print("Start Chestnut-Picker-ROS node")

		### Delta Robot init ###
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

		#### ROS Params ####
		if params_file is None:
			self.Xlength = 880.0 #840.0 #760.0 # This is a true length from camera view
			self.Ylength = 500.0 #480.0 #575.0

			self.XLeftLimit = -260.0
			self.XRightLimit = 230.0
			self.YUpperLimit = 200.0

			self.grabHeight = -785.0 #-763.0 #-745.0	
			self.XBucket = 0.0
			self.YBucket = -300.0 #-280.0 
			self.ZBucket = -350.0 #-350.0 

			self.cameraOffset = 90.0  #95.0
			#self.roverOffset = 45.0   #according to the throttle speed

			### Speed ###
			self.vx_fwd_const = 0.1 #1120	#1140
			self.vx_bwd_const = 0.05	#870

			### Timer Parameters ###
			self.finishTime = 1200
			self.waitTime = 0.8 #0.5
			self.grab_waitTime = 0.5
			self.goHome_waitTime = 0.6 #1.2

			self.show_log = True


		else:

			f = open(params_file, 'r')
			yamlfile = yaml.load(f)
			rosparam.upload_params("/", yamlfile)
			sv_node = 'delta_robot_params'
			self.Xlength = rosparam.get_param(sv_node+"/Xlength")
			self.Ylength = rosparam.get_param(sv_node+"/Ylength")
			self.XLeftLimit = rosparam.get_param(sv_node+"/XLeftLimit")
			self.XRightLimit = rosparam.get_param(sv_node+"/XRightLimit")
			self.YUpperLimit = rosparam.get_param(sv_node+"/YUpperLimit")
			self.grabHeight = rosparam.get_param(sv_node+"/grabHeight")
			self.XBucket = rosparam.get_param(sv_node+"/XBucket")
			self.YBucket = rosparam.get_param(sv_node+"/YBucket")
			self.ZBucket = rosparam.get_param(sv_node+"/ZBucket")
			self.cameraOffset = rosparam.get_param(sv_node+"/cameraOffset")
			self.vx_fwd_const = rosparam.get_param(sv_node+"/vx_fwd_const")
			self.vx_bwd_const = rosparam.get_param(sv_node+"/vx_bwd_const")
			self.finishTime = rosparam.get_param(sv_node+"/finishTime")
			self.waitTime = rosparam.get_param(sv_node+"/waitTime")
			self.grab_waitTime = rosparam.get_param(sv_node+"/grab_waitTime")
			self.goHome_waitTime = rosparam.get_param(sv_node+"/goHome_waitTime")
			self.show_log = rosparam.get_param(sv_node+"/show_log")


		rospy.loginfo("Using parameters below")
		rospy.loginfo("Xlength: {}".format(self.Xlength))
		rospy.loginfo("Ylength: {}".format(self.Ylength))
		rospy.loginfo("XLeftLimit: {}".format(self.XLeftLimit))
		rospy.loginfo("XRightLimit: {}".format(self.XRightLimit))
		rospy.loginfo("YUpperLimit: {}".format(self.YUpperLimit))
		rospy.loginfo("grabHeight: {}".format(self.grabHeight))
		rospy.loginfo("XBucket: {}".format(self.XBucket))
		rospy.loginfo("YBucket: {}".format(self.YBucket))
		rospy.loginfo("ZBucket: {}".format(self.ZBucket))
		rospy.loginfo("cameraOffset: {}".format(self.cameraOffset))
		rospy.loginfo("vx_fwd_const: {}".format(self.vx_fwd_const))
		rospy.loginfo("vx_bwd_const: {}".format(self.vx_bwd_const))
		rospy.loginfo("finishTime: {}".format(self.finishTime))
		rospy.loginfo("waitTime: {}".format(self.waitTime))
		rospy.loginfo("grab_waitTime: {}".format(self.grab_waitTime))
		rospy.loginfo("goHome_waitTime: {}".format(self.goHome_waitTime))
		rospy.loginfo("show_log: {}".format(self.show_log))

		### Local params ###
		self.ROBOT_MODE = "MANUAL"
		self.picker_flag = True

		### Pub/Sub ###
		rospy.Subscriber("/oakd/detection", DetectionWithDepth, self.detection_callback)
		rospy.Subscriber("/jmoab/sbus_rc_ch", Int16MultiArray, self.sbus_ch_callback)
		rospy.Subscriber("/jmoab/cart_mode", UInt8, self.cart_mode_callback)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.loop()

		rospy.spin()

	#####################
	### ROS Callbacks ###
	#####################
	def detection_callback(self, msg):

		self.nboxes = msg.nboxes

		self.xc = np.array(msg.xc.data)
		self.yc = np.array(msg.yc.data)
		self.closest_depth = np.array(msg.closest_depth.data)

		# print("xc", self.xc)
		# print("yc", self.yc)
		# print("nboxes", self.nboxes)

	def sbus_ch_callback(self, msg):

		## ch6 [5] for pwmcart
		## ch7 [6] for atcart
		if msg.data[6] > 1500:
			self.picker_flag = True
		else:
			self.picker_flag = False

	def cart_mode_callback(self, msg):
		if msg.data == 0:
			self.ROBOT_MODE = "HOLD"
		elif msg.data == 1:
			self.ROBOT_MODE = "MANUAL"
		else:
			self.ROBOT_MODE = "AUTO"

	####################
	### Math Helpers ###
	####################
	def map(self, val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def pixelToCartesian(self, px, py):
		x = self.map(px,0,1280,-self.Xlength/2.0, self.Xlength/2.0) # use map() give similar result as coor_g()
		y = self.map(py,0,720,self.Ylength/2.0,-self.Ylength/2.0)
		return x,y

	def go_pick(self):

		## check it again here, if object is still there
		if self.nboxes > 0:
			X,Y = self.pixelToCartesian(self.xc[0], self.yc[0])
		
			## from_move_flag is telling that the robot has initial speed or not
			## if it has then we add some rover_offset 
			# if from_move_flag:
			# 	Y_with_offset = Y+self.cameraOffset+self.roverOffset
			# else:
			# 	Y_with_offset = Y+self.cameraOffset
			Y_with_offset = Y+self.cameraOffset

			## calculate height
			Z_est = -self.closest_depth[0]*1000.0 - 108.0 # TODO: change this offset for other camera
			if Z_est < self.grabHeight:
				print("use grabHeight")
				Z = self.grabHeight
			else:
				print("use Z_est")
				Z = Z_est

			print("X: {:.2f} | Y: {:.2f} | Y cam off: {:.2f} | Z: {:.2f}".format(\
				X,Y,Y_with_offset, Z))

			## conditions if X,Y in frame area
			X_in_frameZone = (self.XLeftLimit < X < self.XRightLimit)
			Y_in_frameZone = (Y_with_offset < self.YUpperLimit)

			## if Y is in the proper zone
			if (not X_in_frameZone) and (Y_in_frameZone):
				print("X is out frame, and Y is close to frame")
				return
				#continue
			else:
				self.dr.GotoPoint(X,(Y_with_offset), Z)
				time.sleep(self.waitTime)
				self.dr.GripperClose()
				time.sleep(self.grab_waitTime)
				self.dr.GotoPoint(X,(Y_with_offset), self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GotoPoint(self.XBucket, 0.0,self.ZBucket)
				time.sleep(self.waitTime) # self.waitTime/2.0
				self.dr.GotoPoint(self.XBucket,self.YBucket,self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GripperOpen()
				time.sleep(self.grab_waitTime)
				self.dr.GotoPoint(self.XBucket, 0.0,self.ZBucket)
				time.sleep(self.waitTime)
				self.dr.GoHome()
				time.sleep(self.goHome_waitTime)

			# else:
			# 	print("Too close to front frame, go back a bit...")
			# 	sbus_throttle = self.const_bwd_throttle
			# 	sbus_steering = 1024
			# 	self.sbus_cmd.data = [sbus_steering, sbus_throttle]
			# 	self.sbus_cmd_pub.publish(self.sbus_cmd)
			# 	time.sleep(1.0)
		else:
			return

	def publish_cmd_vel(self, vx, wz):
		cmd_vel_msg = Twist()
		cmd_vel_msg.linear.x = vx
		cmd_vel_msg.angular.z = wz
		self.cmd_vel_pub.publish(cmd_vel_msg)


	def loop(self):

		rate = rospy.Rate(20)
		from_pick_flag = False
		last_move_time = time.time()
		j = 0
		move_delay = 1.0

		while not rospy.is_shutdown():

			if (self.nboxes > 0) and (self.picker_flag):
				# print("Found, stop cart")
				vx = 0.0
				wz = 0.0
				self.publish_cmd_vel(vx,wz)
				time.sleep(move_delay)

				# if (time.time() - last_move_time) > move_delay:
				# 	from_move_flag = True
				# else:
				# 	from_move_flag = False

				self.go_pick()


				## Wait a bit after stop
				# if from_move_flag and self.ROBOT_MODE == "AUTO":
				# 	print("wait from move")
				# 	time.sleep(1.5)


				from_move_flag = False
				from_pick_flag = True


			else:
				# print("Going...")
			
				vx = self.vx_fwd_const
				wz = 0.0
				
				self.publish_cmd_vel(vx,wz)

				# if from_pick_flag:
				# 	time.sleep(move_delay)

				from_move_flag = True
				from_pick_flag = False

				last_move_time = time.time()



			rate.sleep()

if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='PWMCart with jmoab_ros')
	parser.add_argument('--params_file',
						help="A file path of DeltaRobotParams.yaml")

	args = parser.parse_args(rospy.myargv()[1:])	# to make it work on launch file
	params_file = args.params_file

	if params_file is None:
		print("No params_file is specified use local parameters")
		yaml_path = None
	else:
		yaml_path = params_file
		print("Use {:}".format(params_file))

	CHP = ChestnutPicker(yaml_path)