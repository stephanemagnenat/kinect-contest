#!/usr/bin/env python
#-*- coding: utf-8 -*-

import roslib
roslib.load_manifest('test2d')
import rospy
from math import *
import tf
import geometry_msgs.msg

baseLinkFrame = '/base_link'
odomFrame = '/odom'
kinectFrame = '/openni_depth'
worldFrame = '/world'
limit_low = 1e-3
limit_high = 1e4

if __name__ == '__main__':
	rospy.init_node('test2d')

	t = tf.TransformListener()

	(a_test, b_test, theta_test) = (0,0,0)

	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		# sleep
		rate.sleep()

		# get parameters from transforms
		curTime = t.getLatestCommonTime(odomFrame, baseLinkFrame)
		lastTime = curTime - rospy.Duration(2)
		print 'curTime: ', curTime, ', lastTime: ', lastTime
		t.waitForTransformFull(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, rospy.Time(3))
		t.waitForTransformFull(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, rospy.Time(3))
		(trans_o, quat_o) = t.lookupTransformFull(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame)
		print 'odom to baselink: ', trans_o, quat_o
		alpha_o = trans_o[0]
		beta_o = trans_o[1]
		theta_o = atan2(quat_o[2], quat_o[3])
		(trans_i, quat_i) = t.lookupTransformFull(kinectFrame, curTime, kinectFrame, lastTime, worldFrame)
		print 'world to kinect: ', trans_i, quat_i
		alpha_i = trans_i[0]
		beta_i = trans_i[1]
		theta_i = atan2(quat_i[2], quat_i[3])
		lastTime = curTime

		# compute correspondances, check values to prevent numerical instabilities
		r_1 = alpha_o * cos(theta_o) + alpha_o + beta_o * sin(theta_o)
		if abs(r_1) < limit_low: continue
		r_2 = beta_o * cos(theta_o) + beta_o - alpha_o * sin(theta_o)
		phi = atan2(r_2, r_1)
		R_denom = 2 * sin(theta_o)
		if abs(R_denom) < limit_low: continue
		R = sqrt(r_1*r_1 + r_2*r_2) / R_denom
		if abs(R) > limit_high: continue
		C_1 = (beta_i + beta_i * cos(theta_o) + alpha_i * sin(theta_o)) / R_denom
		if abs(C_1) < limit_low: continue
		C_2 = (alpha_i + alpha_i * cos(theta_o) + beta_i * sin(theta_o)) / R_denom
		xi = atan2(C_2 + b_test, C_1 + a_test)

		# compute new values
		theta_test = pi - phi - xi
		a_test = R * sin(theta_test + phi) - C_1
		b_test = R * cos(theta_test + phi) - C_2

		if abs(a_test) < limit_low:
			print 'warning, small a_test', a_test
		if abs(b_test) < limit_high:
			print 'warning, small b_test', b_test

		# compute transform
		trans_test = (a_test, b_test, 0)
		quat_test = (0, 0, sin(theta_test / 2), cos(theta_test / 2))
		print 'Estimated transform: trans: ', a_test, ',', b_test, ', rot: ', atan2(quat_test.z, quat_test.w)
