from agrotechsimapi import SimClient
import time
import cv2
import argparse

def main(args):
    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    while is_loop:  
        result = client.get_camera_capture(camera_id = args.camera_num, is_clear = True)
        
        if result is not None and len(result) != 0:
            cv2.imshow("Capture from camera 1", result)

        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        time.sleep(1/30)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    args = parser.parse_args()
    main(args)