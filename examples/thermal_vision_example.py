# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient, CaptureType
import time
import cv2
import argparse

def main(args):
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address = "127.0.0.1", port = 8080)

    # Основной цикл получения тепловизионных изображений
    while is_loop:  
        # Получение кадра с тепловизионной камеры
        # camera_id: 0 - передняя камера, 1 - нижняя камера, 2 - задняя камера
        # type=CaptureType.thermal - указывает на получение тепловизионного изображения
        result = client.get_camera_capture(camera_id = args.camera_num, type = CaptureType.thermal)
        
        # Проверка успешности получения кадра
        if  result is not None:
            if len(result) != 0:
                # Отображение тепловизионного кадра в окне OpenCV
                cv2.imshow(f"Capture from  camera", result)

        # Проверка нажатия клавиши 'q' для выхода из программы
        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        # Задержка для ограничения частоты кадров (30 FPS)
        time.sleep(1/30)


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