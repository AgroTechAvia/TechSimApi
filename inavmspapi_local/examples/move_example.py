from inavmspapi import MultirotorControl  
from inavmspapi.transmitter import TCPTransmitter  
from inavmspapi.msp_codes import MSPCodes

import time

HOST = '127.0.0.1'
PORT = 5762
ADDRESS = (HOST,PORT)

tcp_transmitter = TCPTransmitter(ADDRESS)
tcp_transmitter.connect()
control = MultirotorControl(tcp_transmitter)

time.sleep(2)

#==============ARM MODE====================
msg = control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(0.5)

msg = control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(3)
#==============ARM MODE====================

#==============TAKE OFF=================
msg = control.send_RAW_RC([1500, 1500, 1410, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(2.25)
#==============TAKE OFF=================

#=============MOVE FORWARD==============
msg = control.send_RAW_RC([1500, 1600, 1400, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(4)

msg = control.send_RAW_RC([1500, 1500, 1405, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(1)
#=============MOVE FORWARD==============

#=============MOVE LEFT==============
msg = control.send_RAW_RC([1300, 1450, 1405, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(3)

msg = control.send_RAW_RC([1800, 1470, 1400, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(1)

msg = control.send_RAW_RC([1500, 1500, 1400, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(1)
#=============MOVE LEFT==============

#=============MOVE BACK==============
msg = control.send_RAW_RC([1500, 1400, 1400, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(4)

msg = control.send_RAW_RC([1500, 1600, 1406, 1500, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(1)
#=============MOVE BACK==============

#=====================SPIN======================
msg = control.send_RAW_RC([1500, 1500, 1400, 1700, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(3)

msg = control.send_RAW_RC([1500, 1500, 1400, 1450, 2000, 1000, 1000])
data_handler = control.receive_msg()
time.sleep(0.5)
#=====================SPIN======================

#=============LANDING========================
msg = control.send_RAW_RC([1500, 1500, 1330, 1500, 2000, 1000])
data_handler = control.receive_msg()
time.sleep(0.25)

msg = control.send_RAW_RC([1500, 1500, 1350, 1500, 2000, 1000])
data_handler = control.receive_msg()
time.sleep(0.25)

msg = control.send_RAW_RC([1500, 1500, 1380, 1500, 2000, 1000])
data_handler = control.receive_msg()
time.sleep(0.25)
#=============LANDING========================

