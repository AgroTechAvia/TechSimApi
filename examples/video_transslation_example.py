# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import time
import cv2
import argparse

def main(args):
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address = "127.0.0.1", port = 8080)

    # Запуск потокового видео с камеры
    # port=50051 - порт для gRPC стрима
    # camera_id - номер камеры для трансляции
    # rate - частота кадров для записи видео
    client.start_streaming(port = 50051, camera_id = args.camera_num, rate = args.video_rate)
    
    # Ожидание 30 секунд для трансляции видео
    time.sleep(30)

    # Остановка потокового видео
    client.stop_streaming()


if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора камеры:
    # 0 - передняя камера
    # 1 - нижняя камера  
    # 2 - задняя камера
    # По умолчанию используется передняя камера (0)
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    
    # Добавление аргумента для частоты кадров записываемого видео
    # По умолчанию 30 FPS
    parser.add_argument('--video_rate', type=int, help='Saved video frame rate', default=30)
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)