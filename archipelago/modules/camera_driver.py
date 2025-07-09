from agrotechsimapi import SimClient
import time
import cv2
import argparse
import sys

def cleanup(camera_num):
    
    print(f"\n[Camera] Shutting down {camera_num} camera module ...")
    cv2.destroyAllWindows()

def main(args):
    frequency_ = args.frequency
    camera_num_ = args.camera_num
    
    client = SimClient(address="127.0.0.1", port=8080)
    window_name = f"Capture from camera {camera_num_}"

    try:
        while True:  
            image = client.get_camera_capture(camera_id=camera_num_, is_clear=True)
            
            if image is not None and len(image) != 0:
                cv2.imshow(window_name, image)

            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1 or cv2.waitKey(1) == ord('q'):
                break

            time.sleep(1/frequency_)
            
    except KeyboardInterrupt:
        pass
    finally:
        cleanup(camera_num_)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=int, default=24)
    parser.add_argument('--camera_num', type=int, default=0)

    args = parser.parse_args()
    main(args)
