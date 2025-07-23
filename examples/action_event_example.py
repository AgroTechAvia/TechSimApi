# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
# Импорт модуля для обработки клавиатурных событий
from pynput import keyboard
import time

# Глобальные переменные для управления состоянием программы
client = None
listener = None
is_run = True

def on_i_press(key):
    """Обработчик нажатий клавиш"""
    global client,listener,is_run

    if key.char == 'i':
        # При нажатии клавиши 'i' вызывается событие действия в симуляторе
        print("Call event action")
        client.call_event_action()
    elif key.char == 'o':
        # При нажатии клавиши 'o' завершается работа программы
        is_run = False
    else:
        # Игнорирование других клавиш
        pass

def main():
    """Основная функция программы"""
    global client,listener,is_run

    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address = "127.0.0.1", port = 8080)
    
    # Создание и запуск слушателя клавиатурных событий
    listener = keyboard.Listener(on_press=on_i_press)
    listener.start()

    # Основной цикл программы
    # Программа будет работать до нажатия клавиши 'o'
    while is_run:
        time.sleep(1)
    
    # Остановка слушателя клавиатурных событий при завершении
    listener.stop()
    
if __name__ == "__main__":
    # Запуск основной функции
    main()