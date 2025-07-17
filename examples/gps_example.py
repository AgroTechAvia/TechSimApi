from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

import os
import time

def main():

    HOST = '127.0.0.1'
    PORT = 5762
    ADDRESS = (HOST, PORT)

    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    while True:

        if control.send_RAW_msg(MultirotorControl.MSPCodes['MSP_RAW_GPS'], data=[]):
            dataHandler = control.receive_msg()
            control.process_recv_data(dataHandler)
            print("lat: ", control.GPS_DATA['lat']," lon: ",control.GPS_DATA['lon'])  
            
            """fix : Тип фиксации (0 = нет, 1 = 2D, 2 = 3D)
            numSat : Количество спутников
            lat : Широта (в градусах * 1e7)
            lon : Долгота (в градусах * 1e7)
            alt : Высота (в метрах)
            speed : Скорость (в см/с)
            ground_cours : Курс (в градусах * 10)
            hdop : Точность (только для INAV)"""
            
        
        time.sleep(1/15)

if __name__ == "__main__":
    main()