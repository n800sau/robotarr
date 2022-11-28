#!/usr/bin/env python

import serial, time, struct, math

exec(open('vars.sh').read())

CHAR_EOT = b'\x0a'
CMD_MOVE = b'm'
CMD_STEPS = b's'

# distance between centers of wheels
WHEEL_DIST = 0.176
# full turn distance
FULL_TURN_DIST = WHEEL_DIST * math.pi

# steps per circumference
RSTEPS = 200*16
# diameter in m
WDIAM = 0.06

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
	ser.write(CMD_STEPS)
	ser.write(CHAR_EOT)

	if ser.read(1) == CMD_STEPS:
		v1 = struct.unpack('i', ser.read(4))[0]
		v2 = struct.unpack('i', ser.read(4))[0]
		if ser.read(1) != CHAR_EOT:
			protocol_error()
		print('S1', v1)
		print('S2', v2)
		rs = v1+v2
	else:
		protocol_error()
	return rs

def print_steps():
	while True:
		v = steps(ser)
		if v == 0:
			break

def print_counts(dt):
	t = time.time()
	while time.time() - t < dt:
		steps(ser)

with serial.Serial(DEV, 115200, timeout=0.5) as ser:

	set_speed(ser, 0.1, 0.1)
	print_counts(1)
	set_speed(ser, 0, 0)
	set_speed(ser, -0.1, -0.1)
	print_counts(1)
	set_speed(ser, 0, 0)

	ser.close()
