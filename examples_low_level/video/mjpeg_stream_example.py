# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import cv2
import argparse
from flask import Flask, Response

# Создание Flask приложения
app = Flask(__name__)

# Глобальная переменная для клиента
sim_client = None
camera_id = 0

def generate_frames(cam_id):
    """Генератор кадров для MJPEG стрима"""
    while True:
        try:
            frame = sim_client.get_camera_capture(cam_id)
            if frame is not None:
                # Кодируем кадр в JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            print(f"[ERROR] Ошибка при получении кадра: {e}")
            break


@app.route('/video_feed')
def video_feed():
    """Эндпоинт для MJPEG стрима"""
    return Response(generate_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """Главная страница с инструкцией"""
    return '''
    <h1>TechSim MJPEG Stream</h1>
    <p>Откройте VLC и перейдите по адресу:</p>
    <code>http://localhost:5000/video_feed</code>
    <br><br>
    <p>Или откройте в браузере: <a href="/video_feed">/video_feed</a></p>
    '''


def main(args):
    global sim_client, camera_id
    
    # Создание клиента для подключения к симулятору
    sim_client = SimClient(address="127.0.0.1", port=8080)
    camera_id = args.camera_num
    
    print(f"[INFO] Запуск MJPEG стрима с камеры #{camera_id}")
    print(f"[INFO] Откройте VLC: Media -> Open Network Stream -> http://localhost:5000/video_feed")
    print(f"[INFO] Или откройте в браузере: http://localhost:5000/video_feed")
    print(f"[INFO] Для остановки нажмите Ctrl+C")
    
    # Запуск Flask сервера
    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора камеры:
    # 0 - передняя камера
    # 1 - нижняя камера
    # 2 - задняя камера
    # По умолчанию используется передняя камера (0)
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    
    # Добавление аргумента для порта Flask сервера
    # По умолчанию 5000
    parser.add_argument('--port', type=int, help='Flask server port', default=5000)
    
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)
