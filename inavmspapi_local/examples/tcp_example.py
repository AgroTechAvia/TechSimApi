from inavmspapi import MultirotorControl  
from inavmspapi.transmitter import TCPTransmitter  
from inavmspapi.msp_codes import MSPCodes

import time

HOST = '127.0.0.1'
PORT = 5760
ADDRESS = (HOST,PORT)

tcp_transmitter = TCPTransmitter(ADDRESS)
tcp_transmitter.connect()
control = MultirotorControl(tcp_transmitter)

time.sleep(2)

msg = control.send_RAW_msg(MSPCodes['MSPV2_INAV_STATUS'], data=[])
data_handler = control.receive_msg()

print("data_hendler: {0}".format(data_handler))
data = control.process_recv_data(data_handler)

print("data: {0}".format(data))