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
    
    # Инициализация переменных для управления интенсивностью
    intensity = 0
    step = 0.02

    # Включение первого светодиода при старте
    client.set_led_state(0,True)
    
    # Основной цикл мигания светодиодов
    while is_loop:
        # Выключение обоих светодиодов (led_id 0 и 1)
        client.set_led_state(led_id = 0,new_state = False)
        client.set_led_state(led_id = 1,new_state = False)
        
        # Пауза 0.5 секунды с выключенными светодиодами
        time.sleep(1/2)
        
        # Включение обоих светодиодов
        client.set_led_state(led_id = 0, new_state = True)
        client.set_led_state(led_id = 1, new_state = True)
        
        # Пауза 0.5 секунды с включенными светодиодами
        time.sleep(1/2)

# Запуск основной функции
main()