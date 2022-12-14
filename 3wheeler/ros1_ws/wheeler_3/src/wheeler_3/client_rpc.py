#!/usr/bin/env python3

import socket
import json

class Wheeler:

	def __init__(self, host, port):
		self.host = host
		self.port = port

	def __getattr__(self, method_name):
		def rpc_method(**kwds):
			return self.rpc_request(method_name, **kwds)
		return rpc_method

	def rpc_request(self, method_name, **params):
		data = json.dumps({
			'id': 1,
			'method': method_name,
			"jsonrpc": "2.0",
			'params': params
		})
#		print('request', data)
		# Create a socket (SOCK_STREAM means a TCP socket)
		with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
			sock.connect((self.host, self.port))
			sock.sendall(bytes(data + "\n", "utf-8"))
			rs = json.loads(str(sock.recv(1024), "utf-8"))
			return rs['result']

if __name__ == '__main__':

	import time

	def print_counts(dt):
		t = time.time()
		while time.time() - t < dt:
			print(w.steps())

	HOST, PORT = "localhost", 9999

	w = Wheeler(HOST, PORT)

	for i in range(3):
		print('SS', w.set_speed(v1=0.1, v2 = 0.1))
		print_counts(1)
		print('SS', w.set_speed(v1 = 0, v2 = 0))
		print_counts(1)
		print('SS', w.set_speed(v1 = -0.1, v2 = -0.1))
		print_counts(1)
		print('SS', w.set_speed(v1 = 0, v2 = 0))
