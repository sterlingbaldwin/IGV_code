#!/usr/bin/env python
import socket
import sys

SLAVE = '169.254.159.73'
PORT = 23

"Welcome to the IGV shell! Enter commands to talk to Gir."

s = 0
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SLAVE, PORT))
except:
    print "A connection could not be made"
    sys.exit(1)

while 1:
    cmd = raw_input(">> ")
    if cmd == 'q' or cmd == 'Q':
        s.sendall('ST\r')
        break
    s.sendall(cmd + '\r')
    data = s.recv(1024)
    print data

s.close()
