# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient, CaptureType
import time
import cv2
import cv2.aruco as aruco
import argparse

# Импорт пользовательских модулей для распознавания ArUco маркеров
from aruco_marker_recognizer import ArucoRecognizer
from agrotechsimapi import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix


ARUCO_DICTIONARIES = {
    "4x4_50": aruco.DICT_4X4_50,
    "aruco_original": aruco.DICT_ARUCO_ORIGINAL,
}


def create_recognizers(dictionary_name):
    """Create one or more detectors for the marker dictionaries used by TechSim."""
    if dictionary_name == "auto":
        dictionaries = ARUCO_DICTIONARIES.items()
    else:
        dictionaries = [(dictionary_name, ARUCO_DICTIONARIES[dictionary_name])]

    recognizers = []
    for name, dictionary_id in dictionaries:
        dictionary = aruco_dictionary if name == "4x4_50" else aruco.getPredefinedDictionary(dictionary_id)
        recognizers.append((
            name,
            ArucoRecognizer(
                aruco_dictionary=dictionary,
                marker_size=marker_size,
                distance_coefficients=distance_coefficients,
                detector_parameters=detector_parameters,
                camera_matrix=camera_matrix,
            ),
        ))
    return recognizers


def main(args):
    # Инициализация распознавателя ArUco маркеров с предустановленными параметрами
    # aruco_dictionary - словарь ArUco маркеров (например, DICT_6X6_250)
    # marker_size - физический размер маркера в метрах
    # distance_coefficients - коэффициенты дисторсии камеры
    # detector_parameters - параметры детектора ArUco
    # camera_matrix - внутренние параметры камеры (фокусное расстояние, центр изображения)
    aruco_recognizers = create_recognizers(args.dictionary)
    print(f"[ArUco] Active dictionaries: {', '.join(name for name, _ in aruco_recognizers)}")
    
    # Флаг для управления основным циклом программы
    is_loop = True
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address = "127.0.0.1", port = 8080)
    previous_marker_ids = set()

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
                cv_image_with_markers = None
                markers_ids = None
                dictionary_name = None
                for candidate_name, aruco_recognizer in aruco_recognizers:
                    image_with_markers, candidate_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(result.copy())
                    if candidate_ids is not None:
                        cv_image_with_markers = image_with_markers
                        markers_ids = candidate_ids
                        dictionary_name = candidate_name
                        break

                current_marker_ids = set(markers_ids.flatten().tolist()) if markers_ids is not None else set()
                if current_marker_ids != previous_marker_ids:
                    if current_marker_ids:
                        print(f"[ArUco] Detected IDs {sorted(current_marker_ids)} (dictionary: {dictionary_name})")
                    elif previous_marker_ids:
                        print("[ArUco] Markers lost")
                    previous_marker_ids = current_marker_ids

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
    parser.add_argument(
        '--dictionary',
        choices=['auto', *ARUCO_DICTIONARIES],
        default='auto',
        help='ArUco dictionary: auto (default), 4x4_50, or aruco_original',
    )
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)
