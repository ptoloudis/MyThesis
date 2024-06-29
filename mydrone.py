#!/usr/bin/env python3
import socket
import time
import struct
import threading

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(('',9002))
sock2.bind(('',9005))

droneAddress = None;
# sock.settimeout(0.1)

def receive():
    global droneAddress
    while 1:
        try:
            data,address = sock.recvfrom(100)
        except Exception as ex:
            time.sleep(0.01)
            continue

        droneAddress = address

        sock2.sendto(data,('',9003))  # Org
        sock2.sendto(data,('',9004))  # Unity  

def send():
    global droneAddress
    while 1:
        try:
            data = sock2.recv(1024)
        except Exception as ex:
            time.sleep(0.01)
            continue

        sock.sendto(data,droneAddress)

t1 = threading.Thread(target=receive)
t2 = threading.Thread(target=send)

t1.start()
t2.start()
t1.join()
t2.join()
