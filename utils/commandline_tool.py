#!/usr/bin/env python
import socket
import sys

SLAVE = '169.254.159.73'
PORT = 23

cmd = " ".join(sys.argv[1:]) + '\r'
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((SLAVE, PORT))
s.sendall(cmd)
data = s.recv(1024)
print data

s.close()
