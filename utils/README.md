IGV Utilities
========

This is a composition of different development and debugging tools for the CSU Chico IGV's 
signature robot, the Galil DMC-2183, also known as 'Gir'. These modules are primarily written in Python and some C++.

#### girserve.py
A server that emulates Gir & Gir responses. Echo's commands back to the user as data. Most of the following
python utilities should be able to run with a development server with a '-d' argument.

#### girshell.py
An interactive shell to continously send commands to Gir.

#### girwalker.py
An interaction application to control Gir with up, down, left, and right arrow keys.

#### gircli.py
A very simple command line tool to interface with Gir. Takes a single command for Gir on the command line via command line arguments.

**An important note about this tool: because you're passing arguments to this program on the command line, and Gir takes semicolons ';' as
argument parameters, they must be esacped. If semicolons are not escaped your bash interpreter will interpret everything after the semicolon
as a secondary Linux command.
