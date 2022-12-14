#!/usr/bin/python
# -*- coding: utf-8 -*-

from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

	def __init__(self, robot, logging = False):
		self.robot = robot  # do not delete this line
		self.kp = 0  # k_rho
		self.ka = 0  # k_alpha
		self.kb = 0  # k_beta
		self.logging = logging

		if(logging == True):
			self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])

		self.set_goal_points()

	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		# here is the example of destination code
		
		self.robot.state_des.add_destination(x=100,y=20,theta=0)    #goal point 1
		self.robot.state_des.add_destination(x=190,y=30,theta=1.57) #goal point 2



	def track_point(self):

		# All d_ means destination

		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

		# All c_ means current_

		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration


		# Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		# set new c_v = k_rho*rho, c_w = k_alpha*alpha + k_beta*beta
		
		c_v = 25 #randomly assigned c_v and c_w for demonstration purpose
		c_w = 1.57

		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		self.robot.send_wheel_speed(phi_l = 6.0,phi_r = 6.0) #unit rad/s


		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])

		if abs(c_posX - d_posX) < 90: #you need to modify the reach way point criteria
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				return True
			else:
				print("one goal point reached, continute to next goal point")
		
		return False