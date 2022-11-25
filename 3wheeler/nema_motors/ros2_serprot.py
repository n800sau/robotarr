#!/usr/bin/env python

import serial, time, struct, math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

exec(open('vars.sh').read())

CHAR_EOT = b'\x0a'

# distance between centers of wheels
WHEEL_BASE = 0.176
# full turn distance
FULL_TURN_DIST = WHEEL_BASE * math.pi

# steps per circumference
RSTEPS = 200*16
# diameter in m
WHEEL_DIAM = 0.06

def m2steps(mval):
	circumference = WHEEL_DIAM * math.pi
	return int(RSTEPS * mval / circumference)

def move(ser, fwd=True, turn=0, rpm=10, dist=0.1):
	steps = m2steps(dist)
	ser.write(b'm')
	if fwd:
		sgn = [1, -1]
	else:
		sgn = [-1, 1]
	if turn < 0:
		sgn[0] = sgn[1]
	elif turn > 0:
		sgn[1] = sgn[0]
	ser.write(struct.pack('i', sgn[0] * steps))
	ser.write(struct.pack('i', sgn[1] * steps))
	ser.write(struct.pack('i', rpm))
	ser.write(CHAR_EOT)
	print(ser.read(1))
	print(struct.unpack('i', ser.read(4)))
	print(struct.unpack('i', ser.read(4)))
	print(struct.unpack('i', ser.read(4)))
	print(ser.read(1))

def steps(ser):
	ser.write(b's')
	ser.write(CHAR_EOT)

	print(ser.read(1))
	v1 = struct.unpack('i', ser.read(4))[0]
	v2 = struct.unpack('i', ser.read(4))[0]
	print(v1)
	print(v2)
	print(ser.read(1))
	return v1+v2

def print_steps():
	while True:
		v = steps(ser)
		if v == 0:
			break

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

		self.get_logger().info('I heard: lin %s, ang %s' % (msg.linear.x, msg.angular.x))

		lin_vel_x = msg.linear.x;
		ang_vel = msg.angular.z;

		linear_right = lin_vel_x + ((ang_vel * WHEEL_BASE) / 2.0);
		linear_left = (2.0 * lin_vel_x) - linear_right;
		# 60 means 60 sec in a min
		rpm_right = 60.0 * (linear_right / (math.pi * WHEEL_DIAM));
		rpm_left = 60.0 * (linear_left / (math.pi * WHEEL_DIAM));
		self.get_logger().info('rpm: l %s, r %s' % (rpm_left, rpm_right))
		with serial.Serial(DEV, 115200, timeout=0.5) as ser:
			ser.write(b'v')
			ser.write(struct.pack('i', rpm_right))
			ser.write(struct.pack('i', rpm_left))
			ser.write(CHAR_EOT)
			print(ser.read(1))
			print(struct.unpack('i', ser.read(4)))
			print(struct.unpack('i', ser.read(4)))
			print(ser.read(1))

def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = MinimalSubscriber()


	with serial.Serial(DEV, 115200, timeout=0.5) as ser:

#		move(ser, True)
#		print_steps()
		# turn right
#		move(ser, True, turn=1, dist=FULL_TURN_DIST)
#		print_steps()
		# turn left
#		move(ser, True, turn=-1)
#		print_steps()
#		move(ser, False)
#		print_steps()


		rclpy.spin(minimal_subscriber)

		# Destroy the node explicitly
		# (optional - otherwise it will be done automatically
		# when the garbage collector destroys the node object)
		minimal_subscriber.destroy_node()
		rclpy.shutdown()
		ser.close()

if __name__ == '__main__':
	main()
