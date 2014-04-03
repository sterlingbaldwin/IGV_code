#!/usr/bin/env python
import argparse
import socket
import sys
import os

SLAVE = '169.254.159.73'
PORT = 23

def parse_args():
    parser = argparse.ArgumentParser('A simple CLI tool for interacting with Gir.')
    parser.add_argument('-d', '--development', action = 'store_true', help = 'Uses the development server')
    parser.add_argument('command', metavar = 'CMD',  nargs = '+', help = 'Command to send to Gir')
    return parser.parse_args()

def main():
    args = parse_args()

    if args.command is None:
        print "Error: An empty or nil command can't be sent, giving up."
        sys.exit(1)

    if args.development:
        SLAVE = ''
        PORT = 50000
        if os.system('nc -z localhost 50000') is None:
            print 'The development server is not running, please run girserver and try again.'
            sys.exit(1)

    cmd = ' '.join(args.command) + '\r'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((SLAVE, PORT))
        s.sendall(cmd)
        data = s.recv(1024)
        print data
        s.close()
    except:
        print 'A connection could not be made to the designated server, giving up.'
        s.close()
        sys.exit(1)

    sys.exit(0)

if __name__ == "__main__":
    main()
