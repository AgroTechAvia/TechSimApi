# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import time
import numpy as np

def main():
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address="127.0.0.1", port=8080)

    # Основной цикл получения кинематических данных
    while is_loop:
        # Получение данных о кинематике объекта (положение, ориентация, скорости)
        result = client.get_kinametics_data()
        
        # Вывод данных о местоположении (координаты x, y, z)
        print('location: ', *result['location'])
        
        # Вывод данных об ориентации (углы поворота)
        print('orientation: ', *result['orientation'])
        
        # Вывод данных о линейной скорости (скорости по осям x, y, z)
        print('linear_velocity: ', *result['location'])
        
        # Вывод данных об угловой скорости (скорости вращения)
        print('angular_velocity: ', *result['orientation'])
        
        # Задержка для ограничения частоты обновления данных (50 Hz)
        time.sleep(1/50)

# Запуск основной функции
main()