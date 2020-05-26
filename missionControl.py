# This python script listens on UDP port 1234 
# for messages from the ESP32 board and prints them
import socket
import sys
import struct

try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

try:
    s.bind(('', 1234))
except socket.error , msg:
    print 'Bind failed. Error: ' + str(msg[0]) + ': ' + msg[1]
    sys.exit()

print 'Server listening'

while 1:
    d = s.recvfrom(1024)
    data = d[0]

    if not data:
        break

    print data.strip()
    x1 = raw_input("Enter x1: ")
    k = x1.split(" ")
    p = float(k[0])
    i = float(k[1])
    d2 = float(k[2])
    print(p)
    print(i)
    print(d)
    byte1 = bytearray(struct.pack("d", p))
    byte2 = bytearray(struct.pack("d", i))
    byte3 = bytearray(struct.pack("d", d2))
    data2 = byte1 + byte2 + byte3
    sendBoi = s.sendto(data2, d[1])

s.close()
