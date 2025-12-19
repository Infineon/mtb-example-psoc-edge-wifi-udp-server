#******************************************************************************
# File Name:   udp_client.py
#
# Description: A simple python based UDP client.
# 
#********************************************************************************
# (c) 2024-2025, Infineon Technologies AG, or an affiliate of Infineon
# Technologies AG. All rights reserved.
# This software, associated documentation and materials ("Software") is
# owned by Infineon Technologies AG or one of its affiliates ("Infineon")
# and is protected by and subject to worldwide patent protection, worldwide
# copyright laws, and international treaty provisions. Therefore, you may use
# this Software only as provided in the license agreement accompanying the
# software package from which you obtained this Software. If no license
# agreement applies, then any use, reproduction, modification, translation, or
# compilation of this Software is prohibited without the express written
# permission of Infineon.
# 
# Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
# IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
# THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
# SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
# Infineon reserves the right to make changes to the Software without notice.
# You are responsible for properly designing, programming, and testing the
# functionality and safety of your intended application of the Software, as
# well as complying with any legal requirements related to its use. Infineon
# does not guarantee that the Software will be free from intrusion, data theft
# or loss, or other breaches ("Security Breaches"), and Infineon shall have
# no liability arising out of any Security Breaches. Unless otherwise
# explicitly approved by Infineon, the Software may not be used in any
# application where a failure of the Product or any consequences of the use
# thereof can reasonably be expected to result in personal injury.
#********************************************************************************

#!/usr/bin/env python
import socket
import optparse
import time
import sys


BUFFER_SIZE = 1024

# IP details for the UDP server
DEFAULT_IP   = '192.168.43.154'   # IP address of the UDP server
DEFAULT_PORT = 57345             # Port of the UDP server

START_MSG="A"

def udp_client( server_ip, server_port):
	print("================================================================================")
	print("UDP Client")
	print("================================================================================")
	print("Sending data to UDP Server with IP Address:",server_ip, " Port:",server_port)
    
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.sendto(bytes(START_MSG, "utf-8"), (server_ip, server_port))
	
	while True:
		print("================================================================================")        
		data = s.recv(BUFFER_SIZE);
		print("Command from Server:")
		if data.decode('utf-8') == '0':
			print("LED OFF")
			message = 'LED OFF ACK'
			s.sendto(message.encode('utf-8'), (server_ip, server_port))
		if data.decode('utf-8') == '1':
			print("LED ON")
			message = 'LED ON ACK'
			s.sendto(message.encode('utf-8'), (server_ip, server_port))
		print("Acknowledgement sent to server")        
	
if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option("-p", "--port", dest="port", type="int", default=DEFAULT_PORT, help="Port to listen on [default: %default].")
    parser.add_option("--hostname", dest="hostname", default=DEFAULT_IP, help="Hostname or IP address of the server to connect to.")
    (options, args) = parser.parse_args()
    #start udp client
    udp_client(options.hostname, options.port)
