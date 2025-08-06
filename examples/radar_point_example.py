# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import time
import numpy as np
import argparse

def main(args):
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address="127.0.0.1", port=8080)

    # Основной цикл получения данных радара
    while is_loop:
        # Получение данных с радара
        # radar_id - номер радара: 0(передний)/1(задний)/2(нижний)
        # base_angle - базовый угол сканирования (45 градусов)
        # range_min - минимальная дистанция обнаружения (150 мм)
        # range_max - максимальная дистанция обнаружения (2000 мм)
        # is_clear - очистка предыдущих данных
        # range_error - погрешность измерения дистанции (0.15)
        # angle_error - погрешность измерения угла (0.015)
        result = client.get_radar_point(radar_id=args.radar_num,base_angle=45, range_min=150, range_max=2000,is_clear=True,range_error=0.15,angle_error=0.015)
        
        # Вывод результата сканирования радара
        print(result)
        
        # Задержка для ограничения частоты опроса (30 Hz)
        time.sleep(1/30)

if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора радара:
    # 0 - передний радар
    # 1 - задний радар
    # 2 - нижний радар
    # По умолчанию используется передний радар (0)
    parser.add_argument('--radar_num', type=int, help='Radar number: 0(front)/1(back)/2(bottom)', default=0)
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)