# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient

# Импорт модулей для работы с INAV MSP API (контроль мультикоптера)
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

# Импорт модуля для обработки клавиатурных событий
from pynput import keyboard

import time
import cv2
import argparse

# Импорт пользовательских модулей для распознавания ArUco маркеров
from aruco_marker_recognizer import ArucoRecognizer
from recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

# Глобальные переменные для управления мультикоптером
# Массив значений RC каналов: [Roll, Pitch, Throttle, Yaw, Mode, Aux1, Aux2]
# Значения от 1000 до 2000, 1500 - нейтральное положение
rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
is_control = True  # Флаг активности управления

client = None  # Глобальная переменная для клиента симулятора

def on_press(key):
    """Обработчик нажатий клавиш для телеуправления"""
    global rc_control, is_control, client

    print(f'Key pressed: {key}')  # Добавляем печать нажатой клавиши

    try:
        # Управление по каналу Pitch (наклон вперед/назад)
        if key.char == 'w':
            # Клавиша W - движение вперед (увеличение Pitch)
            rc_control[1] = min(rc_control[1] + 5, 2000)
            print(f'Increased Pitch control: {rc_control[1]}')

        elif key.char == 's':
            # Клавиша S - движение назад (уменьшение Pitch)
            rc_control[1] = max(rc_control[1] - 5, 1000)
            print(f'Decreased Pitch control: {rc_control[1]}')

        # Управление по каналу Roll (наклон влево/вправо)
        elif key.char == 'd':
            # Клавиша D - движение вправо (увеличение Roll)
            rc_control[0] = min(rc_control[0] + 5, 2000)
            print(f'Increased Roll control: {rc_control[0]}')

        elif key.char == 'a':
            # Клавиша A - движение влево (уменьшение Roll)
            rc_control[0] = max(rc_control[0] - 5, 1000)
            print(f'Decreased Roll control: {rc_control[0]}')

        # Управление по каналу Yaw (поворот вокруг вертикальной оси)
        elif key.char == 'e':
            # Клавиша E - поворот вправо (увеличение Yaw)
            rc_control[3] = min(rc_control[3] + 5, 2000)
            print(f'Increased Yaw control: {rc_control[3]}')
            
        elif key.char == 'q':
            # Клавиша Q - поворот влево (уменьшение Yaw)
            rc_control[3] = max(rc_control[3] - 5, 1000)
            print(f'Decreased Yaw control: {rc_control[3]}')

        # Управление по каналу Throttle (газ вверх/вниз)
        elif key.char == 'x':
            # Клавиша X - подъем (увеличение Throttle)
            rc_control[2] = min(rc_control[2] + 5, 2000)
            print(f'Increased Thortle control: {rc_control[2]}')

        elif key.char == 'z':
            # Клавиша Z - снижение (уменьшение Throttle)
            rc_control[2] = max(rc_control[2] - 5, 1000)
            print(f'Decreased Thortle control: {rc_control[2]}')
            
        # Вызов события действия в симуляторе
        elif key.char == 'i':
                print(f'Event action = {client.call_event_action()}')
            
        # Отключение управления
        elif key.char == 'y':
            is_control = False
            print('Control disabled')

    except AttributeError:
        # Обработка специальных клавиш (стрелки, функциональные клавиши и т.д.)
        print(f'Special key {key} pressed')

def main(args):
    """Основная функция программы телеуправления с обнаружением ArUco"""

    # Настройка подключения к INAV
    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    # Создание TCP передатчика для связи с полетным контроллером
    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    # Получение доступа к глобальным переменным
    global rc_control, is_control, client

    # Вывод справки по управлению
    print("Z/X Thortle \nQ/E Yaw \nW/S Pitch \nA/D Roll")

    time.sleep(1)

    # Начальная настройка RC каналов (нейтральное положение)
    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)

    # Включение режима автопилота (RC5 = 2000)
    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()

    # Создание и запуск слушателя клавиатурных событий
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Инициализация распознавателя ArUco маркеров с предустановленными параметрами
    aruco_recognizer = ArucoRecognizer(aruco_dictionary = aruco_dictionary,
                                                marker_size = marker_size,
                                                distance_coefficients = distance_coefficients,
                                                detector_parameters = detector_parameters,
                                                camera_matrix = camera_matrix)
    
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    client = SimClient(address = "127.0.0.1", port = 8080)

    # Основной цикл программы: обработка видео и управление
    while is_loop:  
        # Получение кадра с камеры
        result = client.get_camera_capture(camera_id = args.camera_num)
        
        # Проверка успешности получения кадра
        if  result is not None:
            if len(result) != 0:
                # Обнаружение ArUco маркеров на полученном изображении
                cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(result)

                # Проверка успешности обнаружения маркеров
                if cv_image_with_markers is not None:
                    # Проверка корректности размеров изображения
                    if (cv_image_with_markers.shape[0] > 0) and (cv_image_with_markers.shape[1] > 0):
                        # Использование изображения с отмеченными маркерами для отображения
                        result = cv_image_with_markers
                        
                # Отображение кадра в окне OpenCV
                cv2.imshow(f"Capture from  camera", result)
        
        # Отправка команд управления (если управление активно)
        if is_control:
            control.send_RAW_RC(rc_control)
            control.receive_msg()

        # Проверка нажатия клавиши 'q' для выхода из программы
        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()
            listener.stop()

        # Задержка для ограничения частоты цикла (20 Hz)
        time.sleep(1/20)


if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)