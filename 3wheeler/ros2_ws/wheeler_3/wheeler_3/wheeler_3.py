#!/usr/bin/env python

import serial, time, struct, math, os
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_py as tf2
import tf2_ros
import numpy as np

import time
#import RPi.GPIO as GPIO

CHAR_EOT = b'\x0a'
CMD_MOVE = b'm'
CMD_STEPS = b's'
CMD_RESET = b'r'

# distance between centers of wheels
WHEEL_BASE = 0.176
# full turn distance
FULL_TURN_DIST = WHEEL_BASE * math.pi

# steps per revolation
RSTEPS = 200*16
# diameter in m
WDIAM = 0.06

steps1 = 0
steps2 = 0

def reset_port():
	pass

#	RESET_PIN = 12

	# or, for pin numbering, choose BOARD
#	GPIO.setmode(GPIO.BOARD)

#	GPIO.setwarnings(False)
	#reset
#	GPIO.setup(RESET_PIN, GPIO.OUT)

#	GPIO.output(RESET_PIN, 0)
#	time.sleep(0.2)
#	GPIO.output(RESET_PIN, 1)

def find_hw():
	rs = None
	reset_port()
	time.sleep(3)
	for s in list_ports.comports(include_links=False):
		ser = serial.Serial(s.device, baudrate=9600, timeout=1)
		ser.write(b'pA' + CHAR_EOT)
		cmd = ser.read(1)
		if cmd == b'p':
			if ser.read(1) == b'B':
				if ser.read(1) != CHAR_EOT:
					protocol_error('no eot in reply of ping')
				ser.read(1)
				rs = ser
				break
	return rs

def protocol_error(descr):
	raise Exception('protocol error:' + descr)

def quaternion_from_euler(ai, aj, ak):
	ai /= 2.0
	aj /= 2.0
	ak /= 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q


def vel2freq(mval):
	circumference = WDIAM * math.pi
	return int(mval/circumference * RSTEPS)

def set_speed(ser, v1, v2):
	freq1 = vel2freq(v1)
	freq2 = vel2freq(v2)
#	print('FREQ', freq1, freq2)
	ser.write(CMD_MOVE)
	ser.write(struct.pack('i', freq1))
	ser.write(struct.pack('i', freq2))
	ser.write(CHAR_EOT)
	if ser.read(1) == CMD_MOVE:
		node.get_logger().info('V1:{}'.format(struct.un3pack('i', ser.read(4))[0]))
		node.get_logger().info('V2:{}'.format(struct.un3pack('i', ser.read(4))[0]))
		if ser.read(1) != CHAR_EOT:
			protocol_error('no eot in reply of move')
	else:
		protocol_error('no move command in reply')

def steps(ser):
	global steps1, steps2
	ser.write(CMD_STEPS)
	ser.write(CHAR_EOT)
	if ser.read() == CMD_STEPS:
		v1,v2 = struct.unpack('qq', ser.read(16))[:2]
#		node.get_logger().info('CNT1:{}, CNT2:{}'.format(v1, v2))
		if ser.read(1) != CHAR_EOT:
			protocol_error('no eot in reply of steps')
		dv1 = v1 - steps1
		dv2 = v2 - steps2
		steps1 = v1
		steps2 = v2
#		print('dS1', dv1)
#		print('dS2', dv2)
		rs = (v1, v2)
	else:
		protocol_error('no step command in reply')
	return rs

def reset_steps(ser):
	ser.write(CMD_RESET)
	ser.write(CHAR_EOT)
	if ser.read() == CMD_RESET:
		if ser.read(1) != CHAR_EOT:
			protocol_error('no eot in reply of reset')
	else:
		protocol_error('no reset command in reply')
	steps1 = 0
	steps2 = 0

def m2steps(mval):
	circumference = WHEEL_DIAM * math.pi
	return int(RSTEPS * mval / circumference)

class TheNode(Node):

	def __init__(self):
		super().__init__('wheeler_3')
		self.declare_parameter('serial_port', '')
		serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
		if not serial_port:
			serial_port = find_hw()
		self.get_logger().info('SERIAL PORT %s!!!' % serial_port)
		self.ser = serial.Serial(serial_port, 115200, timeout=1)
		reset_steps(self.ser)
		self.prev_time = self.get_clock().now()
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
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		#
		# initialize the odometry message publisher
		#
		self.odom_pub = self.create_publisher(Odometry, "odom", 10)
		self.subscription_vel = self.create_subscription(TwistStamped, 'cmd_vel', self.vel_callback, 10)
#		self.subscription_vel

	def vel_callback(self, msg):

		self.get_logger().info('I heard: lin %s, ang %s' % (msg.twist.linear.x, msg.twist.angular.z))

		lin_vel_x = msg.twist.linear.x;
		ang_vel = msg.twist.angular.z;

		linear_right = lin_vel_x + ((ang_vel * WHEEL_BASE) / 2.0);
		linear_left = (2.0 * lin_vel_x) - linear_right;
		self.get_logger().info('vel: l %s, r %s' % (linear_left, linear_right))
		set_speed(self.ser, linear_left, linear_right)

	def counts2tf(self):
		counter1,counter2 = steps(self.ser)
		curr_time = self.get_clock().now()
		dt = curr_time - self.prev_time
		dt_sec = dt.nanoseconds / 1e9
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
#		self.get_logger().info("odometry x: {}, y: {}, theta: {}".format(self.x, self.y, self.theta))
		#
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		#
		
		#
		# set the header and translation members
		#
		odom_trans = TransformStamped()
		odom_trans.header.stamp = curr_time.to_msg()
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
		odom.header.stamp = curr_time.to_msg()
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

	global node
	rclpy.init(args=args)

	exit = False

	try:
		node = TheNode()
		while not exit:
			rclpy.spin_once(node, timeout_sec=0.1)
			node.counts2tf()
#			print(time.strftime('%M:%S'))

		# Destroy the node explicitly
		# (optional - otherwise it will be done automatically
		# when the garbage collector destroys the node object)
		node.destroy_node()
		rclpy.shutdown()
	finally:
		node.ser.close()

if __name__ == '__main__':
	main()
