#!/usr/bin/env python3

from serial import Serial

BYTE_EOT = 0x0A.to_bytes(1, 'big')

def read_line(ser):
	count = 0
	reply = b''
	while True:
		char = ser.read()
		if char:
			print('S:%s' % char)
		if char == BYTE_EOT:
			break
		if len(char) == 0:
			return None
		reply += char

	return reply

_locals = {}
print('EOT:%s' % (BYTE_EOT == b'\n'))
exec(open('vars.sh').read(), None, _locals)
ser = Serial(_locals['DEV'], 9600, timeout=0.5, writeTimeout=5)

msg = b'pA' + BYTE_EOT
print('Send:', msg)
ser.write(msg)
print('Got:', ser.read(3))

msg = b'r' + BYTE_EOT
print('Send:', msg)
ser.write(msg)
print('Got:', ser.read(2))

msg = b's' + BYTE_EOT
print('Send:', msg)
ser.write(msg)
print('Got:', ser.read(2+16))
