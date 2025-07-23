# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient, CaptureType
import time
import cv2
import argparse
import os

def main(args):
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address="127.0.0.1", port=8080)
    
    # Параметр для типа спектрального захвата (начиная с 4)
    parameter = 4  
    
    # Директория для сохранения кадров
    output_dir = "saved_frames"
    
    # Создание директории если она не существует
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Основной цикл захвата изображений с различными спектральными параметрами
    while is_loop:
        # Получение кадра с камеры с определенным спектральным параметром
        # camera_id - номер камеры для захвата
        # type=CaptureType(parameter) - тип захвата со спектральным параметром
        result = client.get_camera_capture(camera_id=args.camera_num, type=CaptureType(parameter))
        
        # Проверка успешности получения кадра
        if result is not None and len(result) != 0:
            # Отображение полученного изображения
            cv2.imshow("Capture from camera 1", result)
            cv2.waitKey(1)
            
            # Формирование имени файла с указанием параметра
            filename = os.path.join(output_dir, f"frame_param_{parameter}.png")
            
            # Сохранение изображения на диск
            cv2.imwrite(filename, result)
            print(f"Saved image with param {parameter} with name {filename}")
            
            # Переход к следующему спектральному параметру
            parameter += 1
            
            # Циклический переход обратно к параметру 4 после достижения 9
            if parameter > 9:  
                parameter = 4

        # Задержка между захватами (20 Hz)
        time.sleep(0.05)

if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора камеры:
    # 0 - передняя камера
    # 1 - нижняя камера  
    # 2 - задняя камера
    # По умолчанию используется передняя камера (0)
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)