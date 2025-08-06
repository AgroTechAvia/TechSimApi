from agrotechsimapi import SimClient
from inavmspapi import MultirotorControl, TCPTransmitter
from inavmspapi.msp_codes import MSPCodes

from pynput import keyboard

import time
import argparse

rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
is_control = True
control_step = 5

is_can_call_action_event = False
client = None

def on_press(key):
    global rc_control, is_control,is_can_call_action_event, client

    print(f'Key pressed: {key}')  

    try:
        if is_control == True:
            if key.char == 'w':
                rc_control[1] = min(rc_control[1] + control_step, 2000)
                print(f'Increased Pitch control: {rc_control[1]}')

            elif key.char == 's':
                rc_control[1] = max(rc_control[1] - control_step, 1000)
                print(f'Decreased Pitch control: {rc_control[1]}')

            elif key.char == 'd':
                rc_control[0] = min(rc_control[0] + control_step, 2000)
                print(f'Increased Roll control: {rc_control[0]}')

            elif key.char == 'a':
                rc_control[0] = max(rc_control[0] - control_step, 1000)
                print(f'Decreased Roll control: {rc_control[0]}')

            elif key.char == 'e':
                rc_control[3] = min(rc_control[3] + control_step, 2000)
                print(f'Increased Yaw control: {rc_control[3]}')
                
            elif key.char == 'q':
                rc_control[3] = max(rc_control[3] - control_step, 1000)
                print(f'Decreased Yaw control: {rc_control[3]}')

            elif key.char == 'x':
                rc_control[2] = min(rc_control[2] + control_step, 2000)
                print(f'Increased Thortle control: {rc_control[2]}')

            elif key.char == 'z':
                rc_control[2] = max(rc_control[2] - control_step, 1000)
                print(f'Decreased Thortle control: {rc_control[2]}')

            elif key.char == 'i':
                if is_can_call_action_event == True:
                    print(f'Event action = {client.call_event_action()}')
                
            
        if key.char == 'y':
            is_control = not is_control
            print(f'Control state: {is_control}')
            
            if is_control == False:
                rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
            

    except AttributeError:
        
        print(f'Special key {key} pressed')

def cleanup(control, listener):
    
    print("\n[Teleop] Shutting down teleop module...")

    control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
    control.receive_msg()

    listener.stop()

def main(args):
    is_loop = True
    global is_can_call_action_event,rc_control, is_control, client

    frequency_ = args.frequency
    is_can_call_action_event = True if args.is_action == 'True' else False

    client = SimClient(address="127.0.0.1", port=8080)

    HOST = args.inav_host
    PORT = args.inav_port
    ADDRESS = (HOST, PORT)

    tcp_transmitter = TCPTransmitter(ADDRESS)
    tcp_transmitter.connect()
    control = MultirotorControl(tcp_transmitter)

    print("Z/X Thortle \nQ/E Yaw \nW/S Pitch \nA/D Roll")

    time.sleep(1)

    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)

    control.send_RAW_RC([1000, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    

    try:
        while True:  
            control.send_RAW_RC(rc_control)
            control.receive_msg()
            time.sleep(1/frequency_)
    except KeyboardInterrupt:
        pass  
    finally:
        cleanup(control, listener)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=int, default=20)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)
    parser.add_argument('--is_action', type=str, default='False')

    args = parser.parse_args()
    main(args)