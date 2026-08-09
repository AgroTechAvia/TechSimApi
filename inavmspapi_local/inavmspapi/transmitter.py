from abc import ABC, abstractmethod

from threading import Lock
import logging
import time

import socket
import serial

class Transmitter(ABC):

    @abstractmethod
    def __init__(self):
        self.is_connect = False

    @abstractmethod
    def connect(self):
        pass
    
    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def send(self, bufView, blocking=True, timeout=-1):
        pass

    @abstractmethod
    def receive(self, size, timeout = 10):
        pass

    @abstractmethod
    def local_read(self, size):
        pass


class SerialTransmitter(Transmitter):

    def __init__(self, port: str):

        super().__init__()
        self.write_lock = Lock()
        self.read_lock = Lock()

        self.serial_client = serial.Serial()

        self.serial_client.port = port
        self.serial_client.baudrate = 9600
        self.serial_client.bytesize = serial.EIGHTBITS
        self.serial_client.parity = serial.PARITY_NONE
        self.serial_client.stopbits = serial.STOPBITS_ONE
        self.serial_client.timeout = 1
        self.serial_client.xonxoff = False
        self.serial_client.rtscts = False
        self.serial_client.dsrdtr = False
        self.serial_client.writeTimeout = 1

    def connect(self):

        if self.is_connect is False:

            try:
                self.serial_client.open()
                self.is_connect = True

                logging.info("Serial connect")

            except:
                logging.error("Cant connect to serial")

        else:
            logging.info("Serial_client is connected already")


    def disconnect(self):

        if self.is_connect is True:

            try:
                self.serial_client.close()
                self.is_serial_open = False

                logging.info("Close serial")

            except:
                logging.error("Cant close serial")

        else:
            logging.info("Serial_client is disconnected already")
    
    def send(self, bufView:bytearray, blocking:bool=True, timeout:int=-1):

        if self.write_lock.acquire(blocking, timeout):

            try:
                res = self.serial_client.write(bufView) 

            finally:
                self.write_lock.release()

                if res > 0:
                    logging.info("RAW message sent by serial: {0}".format(bufView))  

                return res
                
    def receive(self, size:int, timeout:int = 10):

        with self.read_lock: 

            local_read = self.serial_client.read
            timeout = time.time() + timeout

            while True:

                if time.time() >= timeout:
                    logging.warning("Timeout occured when receiving a message")
                    break

                msg_header = local_read()

                if msg_header:

                    if ord(msg_header) == 36: 
                        break

            msg = local_read(size - 1) 

            logging.info("Recived msg_header: {0}; msg: {1}".format(msg_header,msg))
            return msg_header, msg
        
    def local_read(self, size:int = 1 ):
        return self.serial_client.read(size)
    

class TCPTransmitter(Transmitter):

    def __init__(self, address):

        super().__init__()

        self.address = address
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):

        if self.is_connect is False:

            try:
                self.tcp_client.connect(self.address)
                self.is_connect = True

                logging.info("tcp connect")

            except:
                logging.error("cant connect to tcp")

        else:
            logging.info("tcp_client is connected already")

    def disconnect(self):

        if self.is_connect is True:

            try:
                self.tcp_client.clsoe()
                self.is_connect = False

                logging.info("close tcp")
            except:
                logging.error("cant close tcp")

        else:
            logging.info("tcp_client is disconnected already")

    def send(self, bufView:bytearray, blocking:bool = True, timeout:int = -1):

        try:
            res = self.tcp_client.send(bufView)
            logging.info("RAW message sent by tcp: {0}".format(bufView))  
            res = 1

        except:
            logging.error("Cant send bufView to tcp")
            res = 0

        return res
    
    def receive(self, size:int, timeout:int = 1):

        try:
            msg_header  = self.tcp_client.recv(1)
            msg =  self.tcp_client.recv(size - 1)

            logging.info("Recived msg_header: {0}; msg: {1}".format(msg_header,msg))
            
            return msg_header, msg
        
        except:
            logging.info("Cant recive msg")
    
    def local_read(self, size = 1):
        return self.tcp_client.recv(size)
    
    

    


        