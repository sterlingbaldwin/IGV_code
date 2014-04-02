#!/usr/bin/env python

import Tkinter as tk
import sys
import socket

SLAVE = '169.254.159.73'
PORT = 23

class App(object):
    def __init__(self):
        self.right = False
        self.left = False
        self.up = False
        self.down = False

    def key_pressed(self, event):
        elif event.keysym == 'Right':
            self.right = True
        elif event.keysym == 'Left':
            self.left = True
        elif event.keysym == 'Up':
            self.up = True
        elif event.keysym == 'down':
	        self.down = True

    def key_released(self, event):
        if event.keysym == 'Right':
            self.right = False
        elif event.keysym == 'Left':
            self.left = False
        elif event.keysym == 'Up':
            self.up = False
        elif event.keysym == 'down':
            self.down = False

    def task(self, socket):
        cmd = ""
        if self.right:
            cmd = 'JG 2000,0;'
            print cmd
        elif self.left:
            cmd = 'JG 0,2000;'
            print cmd
        elif self.up:
            cmd = 'JG 5000,5000;BG;'
            print cmd
        elif self.down:
            cmd = 'ST'
            print cmd

        socket.sendall(cmd + '\r')
        data = socket.recv(1024)
        print data

        root.after(20, self.task, socket)

def main():
    print "Welcome to the IGV walker! Use the WASD keys to take Gir for a walk!\n"

    # Make sure we can establish a connection with Gir
    socket = 0
    try:
        #socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #socket.connect((SLAVE, PORT))
    except:
        print "A connection could not be made, giving up."
        sys.exit(1)

    # Setup keybindings
    app = 0
    root = 0
    try:
        app = Walker()
        root = tk.Tk()

        root.bind_all('<Key>', app.key_pressed)
        root.bind_all('<KeyRelease>', app.key_released)
        root.after(20, app.task, s)
        root.withdraw()
    except:
        print "Could not bind keypress handlers, giving up."

    try:
        root.mainloop()
        root.destroy()
        socket.close()
    except:
        root.destroy()
        socket.close()
        raise

if __name__ == '__main__':
    main()
