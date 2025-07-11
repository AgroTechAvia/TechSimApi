from agrotechsimapi import SimClient, CaptureType
import time
import cv2
import argparse
import os

def main(args):
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)
    parameter = 4  
    output_dir = "saved_frames"
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    while is_loop:
        
        result = client.get_camera_capture(camera_id=args.camera_num, type=CaptureType(parameter))
        
        if result is not None and len(result) != 0:
            cv2.imshow("Capture from camera 1", result)
            cv2.waitKey(1)
            filename = os.path.join(output_dir, f"frame_param_{parameter}.png")
            cv2.imwrite(filename, result)
            print(f"Saved image with param {parameter} with name {filename}")
            
            
            parameter += 1
            if parameter > 9:  
                parameter = 4

        time.sleep(0.05)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    args = parser.parse_args()
    main(args)