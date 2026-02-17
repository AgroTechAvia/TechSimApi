import os
import time
from datetime import datetime
import math
from agrotechsimapi import SimClient, CaptureType

import cv2
import argparse

from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

def get_spectrum_name_split(capture_type):
    return capture_type.name.split('_')[-1]

def create_unique_folder(base_folder):
    folder = base_folder
    counter = 1
    while os.path.exists(folder):
        folder = f"{base_folder}_{counter}"
        counter += 1
    os.makedirs(folder)
    return folder

def create_spectrum_subfolders(main_folder, capture_types):

    subfolders = {}
    for cap_type in capture_types:
        subfolder_name = get_spectrum_name_split(cap_type)
        subfolder_path = os.path.join(main_folder, subfolder_name)
        
        os.makedirs(subfolder_path, exist_ok=True)
        subfolders[cap_type] = subfolder_path
    
    return subfolders

def capture_and_save_image(client, camera_id, Capture_Type, folder, image_prefix):

    result = client.get_camera_capture(camera_id=camera_id, type=Capture_Type)

    if result is not None and len(result) != 0:
        now = datetime.now()
        formatted_time = now.strftime("%Y_%m_%d_%H_%M_%S_%f")[:-4]
        image_name = f"{image_prefix}_{get_spectrum_name_split(Capture_Type)}_{formatted_time}.png"
        file_path = os.path.join(folder, image_name)
        cv2.imwrite(file_path, result)
        print(f"Сохранено изображение: {file_path}")
        return None

def main(folder_name, capture_frequency, capture_types, image_prefix, camera_num, capture_horizontal_speed):

    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    folder = create_unique_folder(folder_name)
    print(f"Папка сохранения: {folder}")
    subfolders = create_spectrum_subfolders(folder, capture_types)

    client = SimClient(address="127.0.0.1", port=8080)
    is_loop = True
    sleep_btw_sperctrum = 0.03

    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)
    
    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.1)
    
    control.send_RAW_RC([1500, 1450, 1400, 1500, 2000, 1000, 1000])
    control.receive_msg()
    time.sleep(5)
    
    control.send_RAW_RC([1500, 1500, 1400, 1500, 2000, 1000, 2000])
    time.sleep(0.5)


    while is_loop:
        try:
            kinematic_result = client.get_kinametics_data()
            vx, vy, vz = kinematic_result['linear_velocity']
            speed_magnitude_horizontal = math.sqrt((vx * 100)**2 + (vy * 100)**2)
            if speed_magnitude_horizontal > capture_horizontal_speed:
                for capture_type in capture_types:
                    capture_and_save_image(client, camera_id=camera_num, Capture_Type=capture_type, folder=subfolders[capture_type], image_prefix=image_prefix)
                    time.sleep(sleep_btw_sperctrum)

        except Exception as e:
            print(f"Error: {str(e)}")

        time.sleep(max(capture_frequency - sleep_btw_sperctrum * len(capture_types), sleep_btw_sperctrum))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder_name', type=str, default="Fields", help='Base folder name')
    parser.add_argument('--file_prefix', type=str, default="Img", help='Image name prefix')
    parser.add_argument('--frequency', type=float, default=1, help='Capture frequency in seconds')
    parser.add_argument('--capture_types', type=CaptureType, nargs='+', default=[CaptureType.spectrum_NIR, CaptureType.spectrum_R], help='List of capture types')
    parser.add_argument('--camera_num', type=int, default=1, help='Camera number: 0(front)/1(bottom)/2(back)')
    parser.add_argument('--horizontal_speed', type=float, default=0.5, help='Minimum capture horizontal speed')
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    
    main(
        folder_name=args.folder_name,
        capture_frequency=args.frequency,
        capture_types=args.capture_types,
        image_prefix=args.file_prefix,
        camera_num=args.camera_num,
        capture_horizontal_speed=args.horizontal_speed
    )