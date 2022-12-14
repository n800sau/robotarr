#!/usr/bin/env python

import socketserver
from jsonrpc import JSONRPCResponseManager, Dispatcher
from serprot import WheelerSerial

class SerProtDispatcher(Dispatcher):

	def __init__(self):
		self.wheeler = WheelerSerial()

	def find_method(self, method_name):
		return getattr(self.wheeler, method_name, None)

	def __getitem__(self, key):
		rs = self.find_method(key)
		if rs is None:
			rs = self.method_map[key]
		return rs

dispatcher = SerProtDispatcher()

class TCPHandler(socketserver.BaseRequestHandler):

	def handle(self):
		# self.request is the TCP socket connected to the client
		self.data = self.request.recv(1024).strip()
#		print("{} wrote:".format(self.client_address[0]))
#		print(self.data)

		# Dispatcher is dictionary {<method_name>: callable}
#		dispatcher["echo"] = lambda s: s
#		dispatcher["add"] = lambda a, b: a + b

		response = JSONRPCResponseManager.handle(self.data, dispatcher)
#		print('response', response.json)
		self.request.sendall(response.json.encode())

if __name__ == "__main__":
	HOST, PORT = "localhost", 9999

	socketserver.TCPServer.allow_reuse_address = True
	with socketserver.TCPServer((HOST, PORT), TCPHandler) as server:
		# Activate the server; this will keep running until you
		# interrupt the program with Ctrl-C
		server.serve_forever()
