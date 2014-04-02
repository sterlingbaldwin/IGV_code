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

s.sendall('PR 5000,5000; AC 50000,50000; SP 5000,5000 \r')
data = s.recv(1024)
print data




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
			cmd = 'BG A;'
			print cmd
   		elif self.left:
			cmd = 'BG B;'
			print cmd
		elif self.up:
			cmd = 'BG A,B;'
			print cmd
    	elif self.down:
			cmd = s.sendall('PR -5000,-5000; AC -50000,-50000; SP -5000,-5000; BG; \r')
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




