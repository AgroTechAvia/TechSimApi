import grpc
import cv2
import numpy as np
from concurrent import futures
import time
import os

from agrotechsimapi import video_pb2, video_pb2_grpc

class VideoStreamService(video_pb2_grpc.VideoStreamServiceServicer):
    def __init__(self):
        self.frame_size = (640, 480)
        self.fps = 30
        self.encoding = 'jpeg'
        self.output_dir = "videos"
        os.makedirs(self.output_dir, exist_ok=True)

    def StreamFrames(self, request_iterator, context):
        client_id = context.peer().replace(":", "_").replace("/", "_")
        filename = os.path.join(self.output_dir, f"record_{int(time.time())}.mp4")
        print(f"[INFO] Client connected: {context.peer()}")
        print(f"[INFO] Saving video to {filename}")

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(filename, fourcc, self.fps, self.frame_size)

        frame_count = 0
        try:
            for frame in request_iterator:
                if frame.encoding != self.encoding:
                    print(f"[WARN] Unsupported encoding: {frame.encoding}")
                    continue

                np_arr = np.frombuffer(frame.data, dtype=np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    out.write(img)
                    frame_count += 1
        except Exception as e:
            print(f"[ERROR] Exception during streaming: {e}")
        finally:
            out.release()
            print(f"[INFO] Client disconnected: {context.peer()}")
            print(f"[INFO] Total frames received: {frame_count}")
            print(f"[INFO] Video saved: {filename}")

        return video_pb2.StreamStatus(message="Stream ended, video saved.")

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    video_pb2_grpc.add_VideoStreamServiceServicer_to_server(VideoStreamService(), server)
    server.add_insecure_port("[::]:50051")
    print("[INFO] Server started on port 50051")
    server.start()
    server.wait_for_termination()

if __name__ == "__main__":
    serve()
