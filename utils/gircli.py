#!/usr/bin/env python
import argparse
import socket
import sys

SLAVE = '169.254.159.73'
PORT = 23

def create_parser():
    parser = argparse.ArgumentParser('A simple CLI tool for interacting with Gir.')
    parser.add_argument('-d', '--development', action = 'store_true', help = 'Uses the development server')
    parser.add_argument('command', metavar = 'CMD', required = True, nargs = '+', help = 'Command to send to Gir')
    return parser

def main():
    parser = create_parser()
    """
    cmd = " ".join(sys.argv[1:]) + '\r'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SLAVE, PORT))
    s.sendall(cmd)
    data = s.recv(1024)
    print data

    s.close()"""
    print parser.parse_args()

if __name__ == "__main__":
    main()
