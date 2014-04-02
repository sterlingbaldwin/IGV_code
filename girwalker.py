import Tkinter as tk
import sys
import socket
SLAVE = '169.254.159.73'
PORT = 23

print "Welcome to the IGV walker! Use the WASD keys to take Gir for a walk!\n"

s = 0
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SLAVE, PORT))
except:
    print "A connection could not be made"
    sys.exit(1)






class App(object):
    def __init__(self):
        self.right = False
        self.left = False
        self.up = False
	self.down = False

    def keyPressed(self,event):
        print "HERE"
	print event.keysym
        if event.keysym == 'Escape':
            root.destroy()
        elif event.keysym == 'Right':
            self.right = True
        elif event.keysym == 'Left':
            self.left = True
        elif event.keysym == 'Up':
            self.up = True
	elif event.keysym == 'down':
	    self.down = True

    def keyReleased(self,event):
        if event.keysym == 'Right':
            self.right = False
        elif event.keysym == 'Left':
            self.left = False
        elif event.keysym == 'Up':
            self.up = False
	elif event.keysym == 'down':
	    self.down = False


    def task(self):
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
    	else:
		cmd = ''
    	s.sendall(cmd + '\r')
    	data = s.recv(1024)
    	print data
    	root.after(20,self.task)


application = App()
root = tk.Tk()
print( "Press arrow key (Escape key to exit):" )

root.bind_all('<Key>', application.keyPressed)
root.bind_all('<KeyRelease>', application.keyReleased)
root.after(20,application.task)

root.mainloop()




