import os
import time
import socket
import logging
import cv2
import argparse
from agrotechsimapi import SimClient, CaptureType
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

# Настройка логирования
logging.basicConfig(level=logging.INFO)

def create_unique_folder(base_folder):
    folder = base_folder
    counter = 1
    while os.path.exists(folder):
        folder = f"{base_folder}_{counter}"
        counter += 1
    os.makedirs(folder)
    return folder

def flush_tcp_buffer(sock):
    """Очищает входящий буфер TCP-сокета"""
    try:
        sock.setblocking(False)
        while True:
            chunk = sock.recv(1024)
            if not chunk:
                break
    except (BlockingIOError, InterruptedError):
        pass
    except Exception as e:
        logging.error(f"Ошибка при очистке буфера: {e}")
    finally:
        sock.setblocking(True)

def get_gps_data(control, client_socket):
    """Получает GPS данные с обработкой ошибок"""
    if control.send_RAW_msg(MSPCodes['MSP_RAW_GPS'], data=[]):
        dataHandler = control.receive_msg()
        
        if dataHandler:
            if not dataHandler.get('packet_error'):
                control.process_recv_data(dataHandler)
                lat = control.GPS_DATA.get('lat')
                lon = control.GPS_DATA.get('lon')
                sats = control.GPS_DATA.get('numSat')
                
                if lat is not None and lon is not None:
                    # Конвертируем из 1e7 в градусы
                    return lat/1e7, lon/1e7, sats
            else:
                logging.warning("Ошибка пакета GPS данных. Очистка буфера...")
                flush_tcp_buffer(client_socket)
                time.sleep(0.5)
    
    return None, None, None

def main(folder_name, capture_frequency, image_prefix, max_images, camera_num, inav_host, inav_port):
    ADDRESS = (inav_host, inav_port)
    
    # Инициализация подключения к контроллеру
    try:
        tcp_transmitter = TCPTransmitter(ADDRESS)
        tcp_transmitter.connect()
        if not tcp_transmitter.is_connect:
            raise ConnectionError("Не удалось подключиться к симулятору")
            
        control = MultirotorControl(tcp_transmitter)
        client_socket = control.transmitter.tcp_client
    except Exception as e:
        logging.error(f"Ошибка подключения: {e}")
        return

    # Инициализация камеры
    client = SimClient(address="127.0.0.1", port=8080)
    
    # Создание папки для изображений
    folder = create_unique_folder(folder_name)
    logging.info(f"Создана папка: {folder}")

    # Последовательность взлета
    try:
        control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.5)
        
        control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(0.1)
        
        control.send_RAW_RC([1500, 1450, 1700, 1500, 2000, 1000, 1000])
        control.receive_msg()
        time.sleep(5)
        
        control.send_RAW_RC([1500, 1500, 1400, 1500, 2000, 1000, 2000])
        time.sleep(0.5)
    except Exception as e:
        logging.error(f"Ошибка при взлете: {e}")
        return

    image_count = 0
    
    while image_count < max_images:
        try:
            # Получаем GPS координаты
            lat, lon, sats = get_gps_data(control, client_socket)
            
            if lat is None or lon is None:
                logging.warning("Не удалось получить GPS данные. Повторная попытка...")
                time.sleep(0.1)
                continue
            
            logging.info(f"Координаты: Широта={lat:.7f}, Долгота={lon:.7f}, Спутники={sats}")

            # Делаем снимок
            result = client.get_camera_capture(camera_id=camera_num, type=CaptureType.color)
            
            if result is not None and len(result) != 0:
                # Формируем имя файла с координатами
                coord_str = f"lat{lat:.7f}_lon{lon:.7f}"
                image_name = f"{image_prefix}_{coord_str}_{image_count + 1:04d}.png"
                file_path = os.path.join(folder, image_name)
                
                # Сохраняем изображение
                cv2.imwrite(file_path, result)
                
                logging.info(f"Сохранено изображение: {file_path}")
                image_count += 1

        except Exception as e:
            logging.error(f"Ошибка: {str(e)}")

        if image_count < max_images:
            time.sleep(capture_frequency)

    logging.info(f"Достигнуто максимальное количество изображений ({max_images}). Завершение работы.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Скрипт для аэрофотосъемки с GPS координатами')
    parser.add_argument('--folder_name', type=str, default="fields", help='Имя базовой папки')
    parser.add_argument('--frequency', type=float, default=0.5, help='Частота съемки в секундах')
    parser.add_argument('--prefix', type=str, default="plan", help='Префикс имени изображений')
    parser.add_argument('--max_images', type=int, default=100, help='Максимальное количество изображений')
    parser.add_argument('--camera_num', type=int, default=1, help='Номер камеры: 0(передняя)/1(нижняя)/2(задняя)')
    parser.add_argument('--inav_host', type=str, default='127.0.0.1', help='Хост INAV симулятора')
    parser.add_argument('--inav_port', type=int, default=5762, help='Порт INAV симулятора')
    
    args = parser.parse_args()
    
    main(
        folder_name=args.folder_name,
        capture_frequency=args.frequency,
        image_prefix=args.prefix,
        max_images=args.max_images,
        camera_num=args.camera_num,
        inav_host=args.inav_host,
        inav_port=args.inav_port
    )