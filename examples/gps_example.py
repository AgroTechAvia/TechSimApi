# Импорт модулей для работы с INAV MSP API (контроль мультикоптера)
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

import os
import time

def main():
    # Настройка сетевого подключения к симулятору
    HOST = '127.0.0.1'  # IP адрес сервера симулятора
    PORT = 5762         # Порт для MSP протокола
    ADDRESS = (HOST, PORT)

    # Создание TCP передатчика для связи с симулятором
    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    
    # Создание объекта управления мультикоптером
    control = MultirotorControl(tcp_transmitter)

    # Основной цикл получения GPS данных
    while True:
        # Отправка запроса на получение сырых GPS данных
        if control.send_RAW_msg(MultirotorControl.MSPCodes['MSP_RAW_GPS'], data=[]):
            # Получение ответа от симулятора
            dataHandler = control.receive_msg()
            
            # Обработка полученных данных
            control.process_recv_data(dataHandler)
            
            # Вывод координат: широта и долгота
            print("lat: ", control.GPS_DATA['lat']," lon: ",control.GPS_DATA['lon'])  
            
            """Описание доступных GPS данных:
            fix : Тип фиксации (0 = нет, 1 = 2D, 2 = 3D)
            numSat : Количество спутников
            lat : Широта (в градусах * 1e7)
            lon : Долгота (в градусах * 1e7)
            alt : Высота (в метрах)
            speed : Скорость (в см/с)
            ground_cours : Курс (в градусах * 10)
            hdop : Точность (только для INAV)"""
            
        # Задержка между запросами GPS данных (15 Hz)
        time.sleep(1/15)

if __name__ == "__main__":
    # Запуск основной функции
    main()