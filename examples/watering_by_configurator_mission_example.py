import os
import time
import socket
import cv2
import argparse
from agrotechsimapi import SimClient, CaptureType
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

    

def main(inav_host, inav_port, watering_time):
    ADDRESS = (inav_host, inav_port)
    
    try:
        tcp_transmitter = TCPTransmitter(ADDRESS)
        tcp_transmitter.connect()
        if not tcp_transmitter.is_connect:
            raise ConnectionError("Не удалось подключиться к симулятору")
            
        control = MultirotorControl(tcp_transmitter)
        client_socket = control.transmitter.tcp_client
    except Exception as e:
        logging.error(f"Ошибка подключения: {e}")
        return

    client = SimClient(address="127.0.0.1", port=8080)
    

    try:
        control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.5)
        
        control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.1)
        
        control.send_RAW_RC([1500, 1450, 1700, 1500, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(5)
        
        control.send_RAW_RC([1500, 1500, 1400, 1500, 2000, 1000, 2000])
        time.sleep(0.5)
        client.call_event_action()
        time.sleep((watering_time*60))
        client.call_event_action()
    except Exception as e:
        logging.error(f"Ошибка при взлете: {e}")
        return

    image_count = 0
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Скрипт для аэрофотосъемки с GPS координатами')
    parser.add_argument('--inav_host', type=str, default='127.0.0.1', help='Хост INAV симулятора')
    parser.add_argument('--inav_port', type=int, default=5762, help='Порт INAV симулятора')
    parser.add_argument('--watering_time', type=int, default=9, help='Время полива(мин)')
    
    args = parser.parse_args()
    
    main(
        inav_host=args.inav_host,
        inav_port=args.inav_port,
        watering_time=args.watering_time
    )