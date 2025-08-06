# Импорт модулей для работы с gRPC сервером и обработки видео
import grpc
import cv2
import numpy as np
from concurrent import futures
import time
import os

# Импорт сгенерированных protobuf модулей для видео стриминга
from agrotechsimapi import video_pb2, video_pb2_grpc

class VideoStreamService(video_pb2_grpc.VideoStreamServiceServicer):
    """Сервис для приема и записи видеопотока через gRPC"""
    
    def __init__(self):
        # Настройки видео
        self.frame_size = (640, 480)  # Размер кадра
        self.fps = 30                 # Частота кадров
        self.encoding = 'jpeg'        # Кодировка кадров
        
        # Создание директории для сохранения видео
        self.output_dir = "videos"
        os.makedirs(self.output_dir, exist_ok=True)

    def StreamFrames(self, request_iterator, context):
        """Обработка потока кадров от клиента"""
        
        # Получение идентификатора клиента из контекста соединения
        client_id = context.peer().replace(":", "_").replace("/", "_")
        
        # Формирование уникального имени файла на основе текущего времени
        filename = os.path.join(self.output_dir, f"record_{int(time.time())}.mp4")
        print(f"[INFO] Client connected: {context.peer()}")
        print(f"[INFO] Saving video to {filename}")

        # Настройка кодека для записи MP4 видео
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(filename, fourcc, self.fps, self.frame_size)

        frame_count = 0
        try:
            # Обработка каждого кадра в потоке
            for frame in request_iterator:
                # Проверка поддерживаемой кодировки
                if frame.encoding != self.encoding:
                    print(f"[WARN] Unsupported encoding: {frame.encoding}")
                    continue

                # Декодирование кадра из байтов в изображение OpenCV
                np_arr = np.frombuffer(frame.data, dtype=np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # Запись кадра в видеофайл если декодирование успешно
                if img is not None:
                    out.write(img)
                    frame_count += 1
        except Exception as e:
            print(f"[ERROR] Exception during streaming: {e}")
        finally:
            # Финализация записи видео
            out.release()
            print(f"[INFO] Client disconnected: {context.peer()}")
            print(f"[INFO] Total frames received: {frame_count}")
            print(f"[INFO] Video saved: {filename}")

        # Возврат статуса завершения
        return video_pb2.StreamStatus(message="Stream ended, video saved.")

def serve():
    """Функция для запуска gRPC сервера"""
    
    # Создание gRPC сервера с пулом потоков
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    
    # Регистрация сервиса видеопотока
    video_pb2_grpc.add_VideoStreamServiceServicer_to_server(VideoStreamService(), server)
    
    # Запуск сервера на порту 50051 (доступен для всех интерфейсов)
    server.add_insecure_port("[::]:50051")
    print("[INFO] Server started on port 50051")
    
    # Запуск и ожидание завершения сервера
    server.start()
    server.wait_for_termination()

if __name__ == "__main__":
    # Запуск сервера записи видео
    serve()
