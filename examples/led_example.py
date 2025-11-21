# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import time
import numpy as np
import sys

def set_led(client, rgb):
    r, g, b = rgb
    client.set_Diod(float(r), float(g), float(b))

def main():
    # Флаг для управления основным циклом программы
    is_loop = True

    # Стандартное значение цвета RGB
    color = [0, 0, 255]
    color_black = [0, 0, 0]

    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address="127.0.0.1", port=8080)

    # Включение первого светодиода при старте
    set_led(client, color)
    
    # Основной цикл мигания светодиода
    while is_loop:
        # Выключение
        set_led(client, color_black)
        print(color_black)

        # Пауза 0.5 секунды с выключенным светодиодом
        time.sleep(1/2)    

        # Включение
        set_led(client, color)
        print(color)

        # Пауза 0.5 секунды с выключенным светодиодом
        time.sleep(1/2)

# Запуск основной функции
main()