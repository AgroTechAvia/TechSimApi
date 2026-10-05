from inavmspapi import MultirotorControl 
from inavmspapi.msp_codes import MSPCodes
from inavmspapi.transmitter import SerialTransmitter

import time

ADDRESS = "COM6"


serial_transmitter = SerialTransmitter(ADDRESS)
serial_transmitter.connect()
control = MultirotorControl(serial_transmitter)

time.sleep(2)

msg = control.send_RAW_msg(MSPCodes['MSPV2_INAV_STATUS'], data=[])
data_handler = control.receive_msg()


print("data_hendler: {0}".format(data_handler))
data = control.process_recv_data(data_handler)

print("data: {0}".format(data))