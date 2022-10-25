#!/usr/bin/python
# -*- coding: utf-8 -*-

from calendar import c
from sys import float_repr_style
from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

	def __init__(self, robot, logging = False):
		self.robot = robot  # do not delete this line
		self.kp = 5  # k_rho
		self.ka = 12  # k_alpha
		self.kb = -1.5  # k_beta
		self.finish = 1
		self.fltrC_w = 50
		self.fltrC_v = 100
		self.logging = logging

		if(logging == True):
			#self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])
			#self.robot.make_headers(['pos_X','rho', 'd_theta', 'alpha', 'beta'])
			self.robot.make_headers(['pos_X','pos_y','vix','viy','wi','c_theta','goal_X','goal_y','d_theta'])
		self.set_goal_points()

	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		# here is the example of destination code
		
		self.robot.state_des.add_destination(x=120,y=130,theta=-0.43)  #goal point 1
		self.robot.state_des.add_destination(x=190,y=-0,theta=2.1)     #goal point 2
		self.robot.state_des.add_destination(x=-150,y=-175,theta=0)    #goal point 3
		self.robot.state_des.add_destination(x=-150,y=-25,theta=-1.57) #goal point 4
		self.robot.state_des.add_destination(x=-25,y=200,theta=2.5)    #goal point 5


	def track_point(self):

		# All d_ means destination

		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

		# All c_ means current_

		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration


		# Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		rho=math.sqrt((d_posX-c_posX)**2+(d_posY-c_posY)**2)
		omega=math.atan2(d_posY-c_posY,d_posX-c_posX)
		alpha=omega-c_theta
		
        # Determine if forward or backwards method is to be used
		if (- math.pi/2 < alpha <= math.pi/2):
			beta = d_theta - omega # beta=omega-d_theta
			c_v= self.kp*rho
		else:
			omega=math.atan2(d_posY-c_posY,c_posX-d_posX)
			beta= -(omega + d_theta)  # beta= omega + d_theta
			alpha =  c_theta + beta # - c_theta - beta
			c_v= -self.kp*rho
		
        # Saturation filter for velocity
		if c_v > self.fltrC_v: c_v = self.fltrC_v
		if c_v < -self.fltrC_v: c_v = -self.fltrC_v

		c_w=self.ka*alpha + self.kb*beta
		# Saturation filter for angular velocity
		if abs(c_w) > self.fltrC_w:
			if abs(c_w) == self.fltrC_w:
				c_w = self.fltrC_w
			else:
				c_w = -self.fltrC_w
		
		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		# my fun attempt Deas
		phi_l = (1/3)*c_v +4*c_w
		phi_r = (1/3)*c_v -4*c_w
		
		if phi_l > 16: phi_l =16

		if phi_l < -16: phi_l = -16

		if phi_r > 16: phi_r = 16

		if phi_r < -16: phi_r = -16

		self.robot.send_wheel_speed(float("{:.1f}".format(phi_l)),float("{:.1f}".format(phi_r))) #unit rad/s phi_l = 6.0,phi_r = 6.0


		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			#self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])
			#self.robot.log_data([ c_posX , rho , d_theta, alpha, beta ])
			self.robot.log_data([c_posX,c_posY,c_vix,c_viy,c_wi,c_theta,d_posX,d_posY,d_theta ])

		if abs(rho) < self.finish and abs(d_theta - c_theta) < 0.1: #you need to modify the reach way point criteria  if abs(c_posX - d_posX) < 80:
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				self.robot.send_wheel_speed(.0, .0)
				return True
			else:
				print("one goal point reached, continute to next goal point")
		
		return False
