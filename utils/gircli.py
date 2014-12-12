#!/usr/bin/env python
import argparse
import socket
import sys
import os

SLAVE = '0.0.0.0'
PORT = 23

def parse_args():
    parser = argparse.ArgumentParser('A simple CLI tool for interacting with Gir.')
    parser.add_argument('-d', '--development', action = 'store_true', help = 'Uses the development server')
    parser.add_argument('command', metavar = 'CMD',  nargs = '+', help = 'Command to send to Gir')
    return parser.parse_args()

def main():
    args = parse_args()

    if args.development:
        SLAVE = ''
        PORT = 50000
        if os.system('nc -z localhost 50000') is None:
            print 'The development server is not running, please run girserver and try again.'
            sys.exit(1)

    cmd = ' '.join(args.command) + '\r'
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((SLAVE, PORT))
        sock.sendall(cmd)
        data = sock.recv(1024)
        print data
        sock.close()
    except:
        print 'A connection could not be made to the designated server, giving up.'
        sock.close()
        sys.exit(1)

    sys.exit(0)

if __name__ == "__main__":
    main()
