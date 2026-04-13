# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import cv2
import subprocess
import threading
import time
import argparse
import shutil
import sys


def check_ffmpeg():
    """Проверка наличия FFmpeg в PATH"""
    if shutil.which('ffmpeg') is None:
        print("[ERROR] FFmpeg не найден в PATH!")
        print("[INFO] Установите FFmpeg:")
        print("  - Windows: скачайте с https://ffmpeg.org/download.html и добавьте в PATH")
        print("  - Или используйте MJPEG пример (mjpeg_stream_example.py), который не требует FFmpeg")
        sys.exit(1)
    print("[OK] FFmpeg найден")


class RTSPBridge:
    """Мост для трансляции видео из симулятора через RTSP"""
    
    def __init__(self, camera_id=0, fps=30, rtsp_port=8554):
        self.client = SimClient(address="127.0.0.1", port=8080)
        self.camera_id = camera_id
        self.fps = fps
        self.rtsp_port = rtsp_port
        self.running = False
        self.ffmpeg_proc = None
        self.frame_shape = None

    def start_ffmpeg(self, width, height):
        """Запуск FFmpeg процесса для RTSP трансляции"""
        cmd = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', str(self.fps),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-f', 'rtsp',
            f'rtsp://0.0.0.0:{self.rtsp_port}/stream'
        ]
        
        print(f"[INFO] Запуск FFmpeg: {' '.join(cmd)}")
        self.ffmpeg_proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

    def stream(self):
        """Основной метод для стриминга"""
        print(f"[INFO] Подключение к симулятору...")
        
        # Получаем первый кадр для определения разрешения
        test_frame = self.client.get_camera_capture(self.camera_id)
        if test_frame is None:
            print("[ERROR] Не удалось получить кадр от симулятора")
            return
        
        self.frame_shape = test_frame.shape
        height, width = test_frame.shape[:2]
        
        print(f"[INFO] Получен кадр: {width}x{height}")
        print(f"[INFO] Запуск RTSP стрима на порту {self.rtsp_port}")
        print(f"[INFO] Откройте VLC: rtsp://localhost:{self.rtsp_port}/stream")
        
        # Запускаем FFmpeg
        self.start_ffmpeg(width, height)
        
        self.running = True
        frame_interval = 1.0 / self.fps
        
        try:
            while self.running:
                start_time = time.time()
                
                # Получаем кадр из симулятора
                frame = self.client.get_camera_capture(self.camera_id)
                
                if frame is not None:
                    # Преобразуем из BGRA в BGR (если нужно)
                    if len(frame.shape) == 3 and frame.shape[2] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                    
                    try:
                        # Отправляем кадр в FFmpeg
                        self.ffmpeg_proc.stdin.write(frame.tobytes())
                    except (BrokenPipeError, OSError) as e:
                        print(f"[ERROR] Ошибка записи в FFmpeg: {e}")
                        break
                
                # Контролируем FPS
                elapsed = time.time() - start_time
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("[INFO] Получен сигнал остановки")
        finally:
            self.stop()

    def stop(self):
        """Остановка стриминга"""
        self.running = False
        print("[INFO] Остановка RTSP стрима...")
        
        if self.ffmpeg_proc and self.ffmpeg_proc.stdin:
            try:
                self.ffmpeg_proc.stdin.close()
            except:
                pass
        
        if self.ffmpeg_proc:
            try:
                self.ffmpeg_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.ffmpeg_proc.kill()
                print("[WARN] FFmpeg процесс был принудительно остановлен")
        
        print("[INFO] Стрим остановлен")


def main(args):
    # Проверяем наличие FFmpeg
    check_ffmpeg()
    
    # Создаём мост для трансляции
    bridge = RTSPBridge(
        camera_id=args.camera_num,
        fps=args.fps,
        rtsp_port=args.rtsp_port
    )
    
    # Запускаем стриминг
    bridge.stream()


if __name__ == "__main__":
    # Настройка парсера аргументов командной строки
    parser = argparse.ArgumentParser()
    
    # Добавление аргумента для выбора камеры:
    # 0 - передняя камера
    # 1 - нижняя камера
    # 2 - задняя камера
    # По умолчанию используется передняя камера (0)
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    
    # Добавление аргумента для FPS
    parser.add_argument('--fps', type=int, help='Stream frames per second', default=30)
    
    # Добавление аргумента для порта RTSP
    parser.add_argument('--rtsp_port', type=int, help='RTSP server port', default=8554)
    
    args = parser.parse_args()
    
    # Запуск основной функции с переданными аргументами
    main(args)
