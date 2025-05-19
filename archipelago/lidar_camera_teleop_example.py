from agrotechsimapi import SimClient
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

from pynput import keyboard

import time
import cv2
import argparse
import numpy as np
import matplotlib.pyplot as plt

rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
is_control = True

def on_release(key):
    global rc_control, is_control

    try:
        if key.char == 'w' or key.char == 's': 
            rc_control[1] = 1500

        if key.char == 'd' or key.char == 'a': 
            rc_control[0] = 1500

        if key.char == 'q' or key.char == 'e':   
            rc_control[3] = 1500

        if key.char == 'z' or key.char == 'x': 
            rc_control[2] = 1399

    except AttributeError:
        print(f'Special key {key} pressed')

def on_press(key):
    global rc_control, is_control

    print(f'Key pressed: {key}')  

    try:
        if key.char == 'w':
            
            rc_control[1] = 1900
            print(f'Increased Pitch control: {rc_control[1]}')
        elif key.char == 's':
            rc_control[1] = 1100
            print(f'Decreased Pitch control: {rc_control[1]}')
 
        elif key.char == 'd':
            rc_control[0] = 1900
            print(f'Increased Roll control: {rc_control[0]}')
  
        elif key.char == 'a':
            rc_control[0] = 1100
            print(f'Decreased Roll control: {rc_control[0]}')
  
        elif key.char == 'e':
            rc_control[3] = 1700
            print(f'Increased Yaw control: {rc_control[3]}')
   
        elif key.char == 'q':
            rc_control[3] = 1300
            print(f'Decreased Yaw control: {rc_control[3]}')
   
        elif key.char == 'x':
            rc_control[2] = 1490
            print(f'Increased Thortle control: {rc_control[2]}')

        elif key.char == 'z':
            rc_control[2] = 1350
            print(f'Decreased Thortle control: {rc_control[2]}')

        elif key.char == 'y':
            is_control = False
            print('Control disabled')

    except AttributeError:
        # Для специальных клавиш
        print(f'Special key {key} pressed')

def plot_lidar_data(distances):

    angles = np.linspace(-np.pi, np.pi, num=len(distances), endpoint=False)
    

    x = distances * -np.cos(angles + np.pi/2)
    y = distances * np.sin(angles + np.pi/2)
    
    plt.clf()  
    plt.scatter(x, y, s=5)  
    plt.ylim(-12, 12)  
    plt.xlim(-12, 12)  
    plt.title("Lidar Scan Data")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.pause(0.1) 

def main(args):

    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    global rc_control, is_control

    print("Z/X Thortle \nQ/E Yaw \nW/S Pitch \nA/D Roll")

    time.sleep(1)

    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)

    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    plt.figure()  
    while is_loop:  
        image = client.get_camera_capture(camera_id = args.camera_num, is_clear=True)

        if  image is not None:
            if image is not None and len(image) != 0:
                cv2.imshow("Capture from camera 1", image)

        scan = client.get_laser_scan(angle_min=-np.pi, angle_max=np.pi, range_max=10, num_ranges=360, range_error=0.1)
        plot_lidar_data(scan)

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