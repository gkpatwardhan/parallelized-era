#!/usr/bin/python

import sys
import rosbag
import socket
import struct

HOST  = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT1 = 5558         # Port to listen on & receive from
PORT2 = 5561         # Port to listen on & send to

def recvall(sock, n):
        # Helper function to recv all 'n' bytes of a message
        data = bytearray()
        while len(data) < n :
                packet = sock.recv(n - len(data))
                if not packet:
                        return None
                data.extend(packet)
        return data
        

def main():
        msg_count = 0
        
	s1 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	s1.bind((HOST, PORT1))
	s1.listen(1)
	conn1, addr1 = s1.accept()
	print('Connected to ' + str(addr1))

	s2 =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

	s2.bind((HOST, PORT2))
	s2.listen(1)
	conn2, addr2 = s2.accept()
	print('Connected to ' + str(addr2))

        while True :
		header = recvall(conn1, 8) #conn1.recv(8)
                if not header :
                        print('... end of run on message %d.' % msg_count)
                        break
		print('Wifi1 msg %d received %d bytes from port %d' % (msg_count, len(header), PORT1))
                print('   msg: "%s"' % str(header))

                d_len = int(header[1:7])
                print('    Therefore %d bytes for real/imag parts' % d_len)
		d_real = recvall(conn1, d_len) # conn1.recv(d_len)
                if not d_real :
                        print('... end of run on d_real of message %d.' % msg_count)
                        break
		print('Wifi1 msg %d received %d bytes from port %d' % (msg_count, len(d_real), PORT1))
                #print('   msg: "%s"' % str(d_real))

		d_imag = recvall(conn1, d_len) # conn1.recv(d_len)
                if not d_imag :
                        print('... end of run on d_imag of message %d.' % msg_count)
                        break
		print('Wifi1 msg %d received %d bytes from port %d' % (msg_count, len(d_imag), PORT1))
                #print('   msg: "%s"' % str(d_imag))

		print('Wfi1-1 sending messages %d to port %d' % (msg_count, PORT2))
                conn2.sendall(header)
                conn2.sendall(d_real)
                conn2.sendall(d_imag)
		print('Wfi1-1 sent all messages %d to port %d' %(msg_count, PORT2))
                msg_count += 1

if __name__ == "__main__":
	main()