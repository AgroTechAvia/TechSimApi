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

    # Основной цикл получения данных дальномера
    while is_loop:
        # Получение данных с дальномера
        # rangefinder_id - номер дальномера: 0(передний)/1(задний)/2(нижний)
        # range_min - минимальная дистанция измерения (0.15 м)
        # range_max - максимальная дистанция измерения (10 м)
        # is_clear - очистка предыдущих данных
        # range_error - погрешность измерения (0.15)
        result = client.get_range_data(rangefinder_id = args.range_fire_num, range_min = 0.15, range_max = 10, is_clear = True, range_error = 0.15)
        
        # Вывод результата измерения дистанции
        print(result)
        
        # Задержка для ограничения частоты опроса (30 Hz)
        time.sleep(1/30)

if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора дальномера:
    # 0 - передний дальномер
    # 1 - задний дальномер
    # 2 - нижний дальномер
    # По умолчанию используется передний дальномер (0)
    parser.add_argument('--range_fire_num', type=int, help='Range fire number: 0(front)/1(back)/2(bottom)', default=0)
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)