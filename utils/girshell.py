#!/usr/bin/env python
import argparse
import socket
import sys

SLAVE = '169.254.159.73'
PORT = 23

def create_parser():
    parser = argparse.ArgumentParser('A simple CLI tool for interacting with Gir.')
    parser.add_argument('-d', '--development', action = 'store_true', help = 'Uses the development server')
    parser.add_argument('cmd', 'command', nargs = ? help = 'Command to send to Gir')
    return parser


def main():
    print "Welcome to the IGV shell! Enter commands to talk to Gir."

    parser = create_parser()

    sock = 0
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SLAVE, PORT))
    except:
        print "A connection could not be made"
        sys.exit(1)

    try:
        while 1:
            cmd = raw_input(">> ")
            if cmd == 'q' or cmd == 'Q':
                s.sendall('ST\r')
                break
            sock.sendall(cmd + '\r')
            data = sock.recv(1024)
            print data

        s.close()
    except KeyboardInterrupt:
        if sock: sock.close()
    except:
        if sock: sock.close()
        raise

if __name__ == '__main__':
    main()
