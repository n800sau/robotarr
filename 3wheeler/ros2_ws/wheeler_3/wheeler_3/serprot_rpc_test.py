#!/usr/bin/env python

import socket
import json
import time

HOST, PORT = "localhost", 9999

def rpc_request(method_name, **params):
	data = json.dumps({
		'id': 1,
		'method': method_name,
		"jsonrpc": "2.0",
		'params': params
	})
	print('request', data)
	# Create a socket (SOCK_STREAM means a TCP socket)
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
		sock.connect((HOST, PORT))
		sock.sendall(bytes(data + "\n", "utf-8"))
		rs = json.loads(str(sock.recv(1024), "utf-8"))
		return rs

def print_counts(dt):
	t = time.time()
	while time.time() - t < dt:
		print(rpc_request('steps'))

for i in range(3):
	rpc_request('set_speed', v1=0.1, v2 = 0.1)
	print_counts(1)
	rpc_request('set_speed', v1 = 0, v2 = 0)
	print_counts(1)
	rpc_request('set_speed', v1 = -0.1, v2 = -0.1)
	print_counts(1)
	rpc_request('set_speed', v1 = 0, v2 = 0)
