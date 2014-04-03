#!/usr/bin/env python
import socket

def main():
    sock = 0
    client = 0
    try:
        host, port, max_con = '', 50000, 2
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        sock.bind((host, port)) 
        sock.listen(max_con) 
        while 1: 
            client, address = sock.accept() 
            data = client.recv(1024) 
            if data: 
                client.send(data) 
                client.close()
        sock.close()
        client.close()
    except KeyboardInterrupt:
        if sock: sock.close()
        if client: client.close()
    except:
        if sock: sock.close()
        if client: client.close()
        raise

if __name__ == '__main__':
    main()
