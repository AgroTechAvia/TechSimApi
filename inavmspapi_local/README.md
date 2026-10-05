# InavMSPApi

## Main information

To work with SMEs, you must select a transmitter. Then you need to pass the address for the connection to the transmitter as a parameter. Connect and pass the transmitter as a parameter to an instance of the MultirotorControl class.


## Examples
### Example Serial

 ```python
from inavmspapi import MultirotorControl 
from inavmspapi.transmitter import SerialTransmitter
from inavmspapi.msp_codes import MSPCodes

import time

ADDRESS = "COM6"

MSPV2_INAV_STATUS = 0x2000

serial_transmitter = SerialTransmitter(ADDRESS)
serial_transmitter.connect()
control = MultirotorControl(serial_transmitter)

time.sleep(2)

msg = control.send_RAW_msg(MSPV2_INAV_STATUS, data=[])
data_handler = control.receive_msg()

print("data_hendler: {0}".format(data_handler))
data = control.process_recv_data(data_handler)

print("data: {0}".format(data))
 ```

 ### Example TCP

 ```python
from inavmspapi import MultirotorControl  
from inavmspapi.transmitter import TCPTransmitter  
from inavmspapi.msp_codes import MSPCodes

import time

HOST = '127.0.0.1'
PORT = 5760
ADDRESS = (HOST,PORT)

MSPV2_INAV_STATUS = 0x2000

tcp_transmitter = TCPTransmitter(ADDRESS)
tcp_transmitter.connect()
control = MultirotorControl(tcp_transmitter)

time.sleep(2)

msg = control.send_RAW_msg(MSPV2_INAV_STATUS, data=[])
data_handler = control.receive_msg()

print("data_hendler: {0}".format(data_handler))
data = control.process_recv_data(data_handler)

print("data: {0}".format(data))

 ```