from agrotechsimapi import SimClient
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

from pynput import keyboard

import time
import cv2
import argparse

from aruco_marker_recognizer import ArucoRecognizer
from recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix


rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
is_control = True

client = None

def on_press(key):
    global rc_control, is_control, client

    print(f'Key pressed: {key}')  # Добавляем печать нажатой клавиши

    try:
        if key.char == 'w':
            rc_control[1] = min(rc_control[1] + 5, 2000)
            print(f'Increased Pitch control: {rc_control[1]}')

        elif key.char == 's':
            rc_control[1] = max(rc_control[1] - 5, 1000)
            print(f'Decreased Pitch control: {rc_control[1]}')

        elif key.char == 'd':
            rc_control[0] = min(rc_control[0] + 5, 2000)
            print(f'Increased Roll control: {rc_control[0]}')

        elif key.char == 'a':
            rc_control[0] = max(rc_control[0] - 5, 1000)
            print(f'Decreased Roll control: {rc_control[0]}')

        elif key.char == 'e':
            rc_control[3] = min(rc_control[3] + 5, 2000)
            print(f'Increased Yaw control: {rc_control[3]}')
            
        elif key.char == 'q':
            rc_control[3] = max(rc_control[3] - 5, 1000)
            print(f'Decreased Yaw control: {rc_control[3]}')

        elif key.char == 'x':
            rc_control[2] = min(rc_control[2] + 5, 2000)
            print(f'Increased Thortle control: {rc_control[2]}')

        elif key.char == 'z':
            rc_control[2] = max(rc_control[2] - 5, 1000)
            print(f'Decreased Thortle control: {rc_control[2]}')
        elif key.char == 'i':
                print(f'Event action = {client.call_event_action()}')
            
        elif key.char == 'y':
            is_control = False
            print('Control disabled')

    except AttributeError:
        
        print(f'Special key {key} pressed')

def main(args):

    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    global rc_control, is_control, client

    print("Z/X Thortle \nQ/E Yaw \nW/S Pitch \nA/D Roll")

    time.sleep(1)

    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)

    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    aruco_recognizer = ArucoRecognizer(aruco_dictionary = aruco_dictionary,
                                                marker_size = marker_size,
                                                distance_coefficients = distance_coefficients,
                                                detector_parameters = detector_parameters,
                                                camera_matrix = camera_matrix)
    
    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    while is_loop:  
        result = client.get_camera_capture(camera_id = args.camera_num, is_clear=True)
        
        if  result is not None:
            if len(result) != 0:
                cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(result)

                if cv_image_with_markers is not None:
                    if (cv_image_with_markers.shape[0] > 0) and (cv_image_with_markers.shape[1] > 0):
                        result = cv_image_with_markers
                        
                cv2.imshow(f"Capture from  camera", result)
        
        if is_control:
            control.send_RAW_RC(rc_control)
            control.receive_msg()

        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()
            listener.stop()

        time.sleep(1/20)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_num', type=int, help='Camera number: 0(front)/1(bottom)/2(back)', default=0)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    main(args)