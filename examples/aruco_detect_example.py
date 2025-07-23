# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient, CaptureType
import time
import cv2
import argparse

# Импорт пользовательских модулей для распознавания ArUco маркеров
from aruco_marker_recognizer import ArucoRecognizer
from recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

def main(args):
    # Инициализация распознавателя ArUco маркеров с предустановленными параметрами
    # aruco_dictionary - словарь ArUco маркеров (например, DICT_6X6_250)
    # marker_size - физический размер маркера в метрах
    # distance_coefficients - коэффициенты дисторсии камеры
    # detector_parameters - параметры детектора ArUco
    # camera_matrix - внутренние параметры камеры (фокусное расстояние, центр изображения)
    aruco_recognizer = ArucoRecognizer(aruco_dictionary = aruco_dictionary,
                                                marker_size = marker_size,
                                                distance_coefficients = distance_coefficients,
                                                detector_parameters = detector_parameters,
                                                camera_matrix = camera_matrix)
    
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address = "127.0.0.1", port = 8080)

    # Основной цикл обработки видеопотока
    while is_loop:  
        # Получение кадра с камеры дрона
        # camera_id: 0 - передняя камера, 1 - нижняя камера, 2 - задняя камера
        result = client.get_camera_capture(camera_id = args.camera_num)
        
        # Проверка успешности получения кадра
        if  result is not None:
            if len(result) != 0:
                # Обнаружение ArUco маркеров на полученном изображении
                # Возвращает: изображение с отмеченными маркерами, ID маркеров, 
                # векторы вращения и перемещения для каждого маркера
                cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(result)

                # Проверка успешности обнаружения маркеров
                if cv_image_with_markers is not None:
                    # Проверка корректности размеров изображения
                    if (cv_image_with_markers.shape[0] > 0) and (cv_image_with_markers.shape[1] > 0):
                        # Использование изображения с отмеченными маркерами для отображения
                        result = cv_image_with_markers
                        
                # Отображение кадра в окне OpenCV
                cv2.imshow(f"Capture from  camera", result)
        

        # Проверка нажатия клавиши 'q' для выхода из программы
        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        # Задержка для ограничения частоты кадров (20 FPS)
        time.sleep(1/20)


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