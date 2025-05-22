from agrotechsimapi import SimClient
import time
import cv2
import argparse

def main(args):
    is_loop = True

    frequency_ = args.frequency
    camera_num_ = args.camera_num

    client = SimClient(address="127.0.0.1", port=8080)


    while is_loop:  
        image = client.get_camera_capture(camera_id = camera_num_, is_clear = True)
        
        if image is not None and len(image) != 0:
            cv2.imshow(f"Capture from camera {camera_num_}", image)

        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        time.sleep(1/frequency_)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=int, default=24)
    parser.add_argument('--camera_num', type=int, default=0)

    args = parser.parse_args()
    main(args)