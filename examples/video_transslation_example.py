from agrotechsimapi import SimClient
import time
import cv2
import argparse

def main(args):
    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    client.start_streaming(port = 50051, camera_id = args.camera_num, rate = args.video_rate)
    
    time.sleep(30)

    client.stop_streaming()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    parser.add_argument('--video_rate', type=int, help='Saved video frame rate', default=30)
    args = parser.parse_args()
    main(args)