#!/usr/bin/python
# -*- coding: utf-8 -*-

from E160_state import *
from E160_robot import *
import math
import time


class controller:

	def __init__(self, robot, logging = False):
		self.robot = robot  # do not delete this line
		self.kp = 0  # k_rho
		self.ka = 0  # k_alpha
		self.kb = 0  # k_beta
		self.logging = logging

		if(logging == True):
			self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])

		self.set_goal_points()
		
		# select the controller type
		self.controller_type='a'
		# self.controller_type='p'
		

	def set_goal_points(self):
		'''
		#Edit goal point list below, if you click a point using mouse, the points programmed
		#will be washed out
		'''
		# here is the example of destination code
		
		self.robot.state_des.add_destination(x=100,y=20,theta=0)    #goal point 1
		self.robot.state_des.add_destination(x=190,y=30,theta=1.57) #goal point 2

	def get_robot_state(self):
		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration
		return c_posX, c_posY, c_theta, c_vix, c_viy, c_wi, c_v, c_w


	def track_point(self):
		'''
		Main controller method for tracking point
		'''
		
		## The following demonstrate how to get the state of the robot

		# All d_ means destination
		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  #get next destination configuration

		# All c_ means current_
		c_posX, c_posY, c_theta, c_vix, c_viy, c_wi, c_v, c_w = self.get_robot_state() #get current robot state

		## Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		
		
		if(self.controller_type=='a'):
			#if it is "a controller", ensure the wheel speed is not violated, you need to
			#1. turn your robot to face the target position
			#2. move your robot forward to the target position
			#3. turn your robot to face the target orientation
			c_v = 25 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
			c_w = 1.57 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
		
		elif(self.controller_type=='p'):
			#use p control discussed in the class, 
			# set new c_v = k_rho*rho, c_w = k_alpha*alpha + k_beta*beta
			c_v = 40 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
			c_w = 1.57 #randomly assigned c_v and c_w for demonstration purpose, remove for your lab!
		
		else:
			print("no controller type is provided")
			c_v = 0
			c_w = 0


		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		self.robot.send_wheel_speed(phi_l = 6.0,phi_r = 6.0) #unit rad/s

		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])


		if abs(c_posX - d_posX)< 5 and (c_theta - d_theta) < 1 : #you need to modify the reach way point criteria
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				return True
			else:
				print("one goal point reached, continute to next goal point")

		return False #just return False, don't remove it.