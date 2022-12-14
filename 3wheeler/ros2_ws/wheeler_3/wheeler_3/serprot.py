#!/usr/bin/env python3

import serial, time, struct, math, os
from serial.tools import list_ports

from reset import reset_stm32

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

BAUD = 9600

def find_hw():
	rs = None
	for s in list_ports.comports(include_links=False):
		print('Testing {}'.format(s))
		with serial.Serial(s.device, baudrate=BAUD, timeout=1) as ser:
			ser.write(b'pA\x0a')
			cmd = ser.read(1)
			if cmd == b'p':
				if ser.read(1) == b'B':
					rs = s.device
					ser.read(1)
					break
	return rs

def init_serial_dev():
	reset_stm32()
	time.sleep(3)

	if os.path.exists('vars.sh'):
		exec(open('vars.sh').read())
	else:
		DEV = find_hw()

	print('DEV', DEV)
	if DEV is None:
		raise Exception('Port not found')

	return serial.Serial(DEV, baudrate=BAUD, timeout=1)

def protocol_error():
	raise Exception('protocol error')

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

def m2steps(mval):
	circumference = WHEEL_DIAM * math.pi
	return int(RSTEPS * mval / circumference)

class WheelerSerial:

	def __init__(self, ser=None):
		if ser is None:
			ser = init_serial_dev()
		self.ser = ser
		self.reset_steps()

	def set_speed(self, v1, v2):
		freq1 = vel2freq(v1)
		freq2 = vel2freq(v2)
#		print('FREQ', freq1, freq2)
		self.ser.write(CMD_MOVE)
		self.ser.write(struct.pack('i', freq1))
		self.ser.write(struct.pack('i', freq2))
		self.ser.write(CHAR_EOT)
		if self.ser.read(1) == CMD_MOVE:
			v1,v2 = struct.unpack('ii', self.ser.read(8))
			if self.ser.read(1) != CHAR_EOT:
				protocol_error()
		else:
			protocol_error()
		return v1,v2

	def steps(self):
		self.ser.write(CMD_STEPS)
		self.ser.write(CHAR_EOT)
		if self.ser.read(1) == CMD_STEPS:
			v1 = struct.unpack('q', self.ser.read(8))[0]
			v2 = struct.unpack('q', self.ser.read(8))[0]
			if self.ser.read(1) != CHAR_EOT:
				protocol_error()
			dv1 = v1 - self.steps1
			dv2 = v2 - self.steps2
			self.steps1 = v1
			self.steps2 = v2
			rs = (v1, v2)
		else:
			protocol_error()
		return rs

	def reset_steps(self):
		self.ser.write(CMD_RESET)
		self.ser.write(CHAR_EOT)
		self.steps1 = 0
		self.steps2 = 0

if __name__ == '__main__':

	def print_steps():
		while True:
			v = wheeler.steps()
			if v == 0:
				break

	def print_counts(dt):
		t = time.time()
		while time.time() - t < dt:
			print(wheeler.steps())

	ser = init_serial_dev()

	try:
		wheeler = WheelerSerial(ser)

		for i in range(3):
			wheeler.set_speed(0.1, 0.1)
			print_counts(1)
			wheeler.set_speed(0, 0)
			print_counts(1)
			wheeler.set_speed(-0.1, -0.1)
			print_counts(1)
			wheeler.set_speed(0, 0)
	finally:
		ser.close()
