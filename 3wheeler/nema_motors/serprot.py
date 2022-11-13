#!/usr/bin/env python

import serial, time, struct, math

exec(open('vars.sh').read())

CHAR_EOT = b'\x0a'

# distance between centers of wheels
WHEEL_DIST = 0.176
# full turn distance
FULL_TURN_DIST = WHEEL_DIST * math.pi

# steps per circumference
RSTEPS = 200*16
# diameter in m
WDIAM = 0.06

def m2steps(mval):
	circumference = WDIAM * math.pi
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

with serial.Serial(DEV, 115200, timeout=0.5) as ser:

#	move(ser, True)
#	print_steps()
	# turn right
	move(ser, True, turn=1, dist=FULL_TURN_DIST)
	print_steps()
	# turn left
#	move(ser, True, turn=-1)
#	print_steps()
#	move(ser, False)
#	print_steps()

	ser.close()
