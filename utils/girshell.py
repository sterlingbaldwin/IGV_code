#!/usr/bin/env python
import argparse
import socket
import sys
import os

SLAVE = '192.168.1.2'
PORT = 23

def parse_args():
    parser = argparse.ArgumentParser('An interactive shell for interfacing with the IVG bot.')
    parser.add_argument('-d', '--development', action = 'store_true', help = 'Uses the development server')
    return parser.parse_args()

def mainloop(sock):
    while (1):
        SLAVE = '192.168.1.2'
        PORT = 23
        cmd = raw_input('>> ')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SLAVE, PORT))
        if cmd == 'quit':
            sock.sendall('ST;\r')    
            sock.close()
            break
        else:
            sock.sendall(cmd + '\r')
        data = sock.recv(1024)
        print data
        sock.close()

def main():
    print "Welcome to the IGV shell! Enter commands to talk to Gir."
    args = parse_args()

    if args.development:
        SLAVE = 'localhost'
        PORT = 50000
    
    SLAVE = '192.168.1.2'
    PORT = 23
    #if os.system('nc -z ' + SLAVE + ' ' + PORT) is None:
    #    print 'Could not create a connection to the designated server, giving up.'
    #    sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        mainloop(sock)
    except KeyboardInterrupt:
        if sock: sock.close()
        sys.exit(0)
    except:
        if sock: sock.close()
        raise

    sys.exit(0)

if __name__ == '__main__':
    main()
