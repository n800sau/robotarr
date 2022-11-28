#!/usr/bin/env python

import serial, time, struct, math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

exec(open('vars.sh').read())

CHAR_EOT = b'\x0a'
CMD_MOVE = b'm'
CMD_STEPS = b's'
CMD_RESET = b'r'

# distance between centers of wheels
WHEEL_BASE = 0.176
# full turn distance
FULL_TURN_DIST = WHEEL_BASE * math.pi

# steps per circumference
RSTEPS = 200*16
# diameter in m
WDIAM = 0.06

steps1 = 0
steps2 = 0

def protocol_error():
	raise Exception('protocol error')

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
		print('V1', struct.unpack('i', ser.read(4))[0])
		print('V2', struct.unpack('i', ser.read(4))[0])
		if ser.read(1) != CHAR_EOT:
			protocol_error()
	else:
		protocol_error()

def steps(ser):
	global steps1, steps2
	ser.write(CMD_STEPS)
	ser.write(CHAR_EOT)

	if ser.read(1) == CMD_STEPS:
		v1 = struct.unpack('q', ser.read(8))[0]
		v2 = struct.unpack('q', ser.read(8))[0]
		if ser.read(1) != CHAR_EOT:
			protocol_error()
		dv1 = v1 - steps1
		dv2 = v2 - steps2
		steps1 = v1
		steps2 = v2
#		print('dS1', dv1)
#		print('dS2', dv2)
		rs = v1+v2
	else:
		protocol_error()
	return rs

def reset_steps(ser):
	ser.write(CMD_RESET)
	ser.write(CHAR_EOT)
	steps1 = 0
	steps2 = 0

def m2steps(mval):
	circumference = WHEEL_DIAM * math.pi
	return int(RSTEPS * mval / circumference)

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription_vel = self.create_subscription(
			Twist,
			'cmd_vel',
			self.vel_callback,
			10)
		self.subscription_vel

	def vel_callback(self, msg):

		self.get_logger().info('I heard: lin %s, ang %s' % (msg.linear.x, msg.angular.z))

		lin_vel_x = msg.linear.x;
		ang_vel = msg.angular.z;

		linear_right = lin_vel_x + ((ang_vel * WHEEL_BASE) / 2.0);
		linear_left = (2.0 * lin_vel_x) - linear_right;
		self.get_logger().info('vel: l %s, r %s' % (linear_left, linear_right))
		with serial.Serial(DEV, 115200, timeout=0.5) as ser:
			set_speed(ser, linear_left, linear_right)

def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = MinimalSubscriber()

	exit = False
	with serial.Serial(DEV, 115200, timeout=0.5) as ser:

		reset_steps(ser)
		while not exit:
			rclpy.spin_once(minimal_subscriber, timeout_sec=0.1)
			steps(ser)
#			print(time.strftime('%M:%S'))

		# Destroy the node explicitly
		# (optional - otherwise it will be done automatically
		# when the garbage collector destroys the node object)
		minimal_subscriber.destroy_node()
		rclpy.shutdown()
		ser.close()

if __name__ == '__main__':
	main()
