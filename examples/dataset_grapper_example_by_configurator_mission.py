# Импорт системных модулей
import os
import time

# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient, CaptureType

import cv2
import argparse

# Импорт модулей для работы с INAV MSP API (контроль мультикоптера)
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

def create_unique_folder(base_folder):
    """Создание уникальной папки для сохранения изображений"""
    folder = base_folder
    counter = 1
    
    # Поиск свободного имени папки
    while os.path.exists(folder):
        folder = f"{base_folder}_{counter}"
        counter += 1
    
    # Создание найденной папки
    os.makedirs(folder)
    return folder

def main(folder_name, capture_frequency, image_prefix, max_images, camera_num):
    """Основная функция сбора данных по миссии конфигуратора"""

    # Настройка подключения к INAV
    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    # Создание TCP передатчика для связи с полетным контроллером
    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    # Создание уникальной папки для сохранения изображений
    folder = create_unique_folder(folder_name)
    print(f"Created folder: {folder}")

    # Создание клиента для подключения к симулятору
    client = SimClient(address="127.0.0.1", port=8080)
    image_count = 0

    # Начальная настройка RC каналов (нейтральное положение)
    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)
    
    # Включение режима автопилота (RC5 = 2000)
    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.1)
    
    # Взлет с установкой значений RC каналов для подъема
    # RC1=1500 (roll), RC2=1450 (pitch назад), RC3=1400 (throttle), RC4=1500 (yaw)
    control.send_RAW_RC([1500, 1450, 1400, 1500, 2000, 1000, 1000])
    control.receive_msg()
    time.sleep(5)
    
    # Активация миссии (RC7 = 2000) при удержании режима автопилота
    control.send_RAW_RC([1500, 1500, 1400, 1500, 2000, 1000, 2000])
    time.sleep(0.5)

    # Основной цикл сбора изображений во время выполнения миссии
    while image_count < max_images:
        try:
            # Получение цветного кадра с камеры
            result = client.get_camera_capture(camera_id=camera_num, type=CaptureType.color)
            
            # Проверка успешности получения кадра
            if result is not None and len(result) != 0:
                # Формирование имени файла с префиксом и номером
                image_name = f"{image_prefix}_{image_count + 1}.png"
                file_path = os.path.join(folder, image_name)
                
                # Сохранение изображения на диск
                cv2.imwrite(file_path, result)
                print(f"Saved image: {file_path}")
                image_count += 1

        except Exception as e:
            print(f"Error: {str(e)}")

        # Задержка между захватами изображений (если не достигнут лимит)
        if image_count < max_images:
            time.sleep(capture_frequency)

    print(f"Reached maximum images count ({max_images}). Program finished.")

if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder_name', type=str, default="fields", help='Base folder name')
    parser.add_argument('--frequency', type=float, default=0.5, help='Capture frequency in seconds')
    parser.add_argument('--prefix', type=str, default="plan", help='Image name prefix')
    parser.add_argument('--max_images', type=int, default=100, help='Maximum number of images')
    parser.add_argument('--camera_num', type=int, default=1, help='Camera number: 0(front)/1(bottom)/2(back)')
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)
    
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(
        folder_name=args.folder_name,
        capture_frequency=args.frequency,
        image_prefix=args.prefix,
        max_images=args.max_images,
        camera_num=args.camera_num
    )