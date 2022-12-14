#!/usr/bin/env python3

import time, struct, math, os

import rospy
import threading
from geometry_msgs.msg import TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_py as tf2
import tf2_ros
import numpy as np

from wheeler_3.wheeler_3_common import *

from wheeler_3.client_rpc import Wheeler

HOST, PORT = "localhost", 9999

class TheNode:

	def __init__(self):
		self.w = Wheeler(HOST, PORT)
		self.prev_time = rospy.Time.now()
		self.theta = 0.
		self.prev_counter1 = 0
		self.prev_counter2 = 0
		self.vth = 0.
		self.vy = 0.
		self.vx = 0.
		self.x = 0.
		self.y = 0.
		#
		# initialize the transform broadcaster
		#
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()
		#
		# initialize the odometry message publisher
		#
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
		self.subscription_vel = rospy.Subscriber('cmd_vel', TwistStamped, self.vel_callback)
#		self.subscription_vel

	def vel_callback(self, msg):

#		rospy.loginfo('{}: I heard: lin {}, ang {}'.format(threading.current_thread(), msg.twist.linear.x, msg.twist.angular.z))

		lin_vel_x = msg.twist.linear.x;
		ang_vel = msg.twist.angular.z;

		linear_right = lin_vel_x + ((ang_vel * WHEEL_BASE) / 2.0);
		linear_left = (2.0 * lin_vel_x) - linear_right;
		rospy.loginfo('vel: l %s, r %s' % (linear_left, linear_right))
		self.w.set_speed(linear_left, linear_right)

	def counts2tf(self):
		counter1,counter2 = self.w.steps()
		# set use_sim_time to false for now to work
		curr_time = rospy.Time.now()
		dt = curr_time - self.prev_time
		dt_sec = dt.to_nsec() / 1e9
#		rospy.loginfo('ctime: {}, dt: {}, sec: {}'.format(curr_time, dt, dt_sec))
		
		#
		# compute odometry in a piecewise linear using small time slices
		#
		# use kinematic model to compute incremental distance change and velocity
		# changes using changes in encoder counts
		# distance = (delta encoder count) / (encoder counts per rev) * pi * wheel diameter
		#
		curr_counter1 = counter1
		curr_counter2 = counter2
		delta_left = (math.pi * WDIAM * (curr_counter1 - self.prev_counter1)) / RSTEPS
		delta_right = (math.pi * WDIAM * (curr_counter2 - self.prev_counter2)) / RSTEPS
		delta_dist = (delta_right + delta_left) / 2.0
		delta_th = (delta_right - delta_left) / WHEEL_BASE
		delta_x = delta_dist * math.cos(self.theta + (delta_th / 2.0))
		delta_y = delta_dist * math.sin(self.theta + (delta_th / 2.0))
		self.theta += delta_th
		self.x += delta_x
		self.y += delta_y
		self.vx = delta_x / dt_sec
		self.vy = delta_y / dt_sec
		self.vth = delta_th / dt_sec
#		rospy.loginfo("odometry x: {}, y: {}, theta: {}".format(self.x, self.y, self.theta))
		#
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		#
		
		#
		# set the header and translation members
		#
		odom_trans = TransformStamped()
		odom_trans.header.stamp = curr_time
		odom_trans.header.frame_id = "odom"
		odom_trans.child_frame_id = "base_link"
		odom_trans.transform.translation.x = self.x
		odom_trans.transform.translation.y = self.y
		odom_trans.transform.translation.z = 0.0
		#
		# populate quaternion members
		#
		tf_quat = quaternion_from_euler(0, 0, self.theta)
		odom_trans.transform.rotation.x = tf_quat[0]
		odom_trans.transform.rotation.y = tf_quat[1]
		odom_trans.transform.rotation.z = tf_quat[2]
		odom_trans.transform.rotation.w = tf_quat[3]
		#
		# Send the transformation
		#
		self.tf_broadcaster.sendTransform(odom_trans)
		#
		# setup and publish the odometry message over ROS
		#
		odom = Odometry()
		odom.header.stamp = curr_time
		odom.header.frame_id = "odom"
		odom_quat = Quaternion(x=tf_quat[0], y=tf_quat[1], z=tf_quat[2], w=tf_quat[3])
		#
		# set the position
		#
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = odom_quat
		#
		# set the velocity
		#
		odom.child_frame_id = "base_link"
		odom.twist.twist.linear.x = self.vx
		odom.twist.twist.linear.y = self.vy
		odom.twist.twist.angular.z = self.vth
		#
		# publish the message
		#
		self.odom_pub.publish(odom)

		self.prev_time = curr_time
		self.prev_counter1 = curr_counter1
		self.prev_counter2 = curr_counter2

def main(args=None):

	rospy.init_node('wheeler_3', anonymous=True)
	node = TheNode()
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		node.counts2tf()
		rate.sleep()

if __name__ == '__main__':
	main()
