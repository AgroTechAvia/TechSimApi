from inavmspapi import MultirotorControl  
from inavmspapi.transmitter import TCPTransmitter  
from inavmspapi.msp_codes import MSPCodes
from pynput import keyboard
import time

HOST = '127.0.0.1'
PORT = 5762
ADDRESS = (HOST, PORT)

tcp_transmitter = TCPTransmitter(ADDRESS)
tcp_transmitter.connect()
control = MultirotorControl(tcp_transmitter)

rc_control = [1500, 1500, 1000, 1500, 2000, 1000, 1000]
is_control = True

# Функция для обработки нажатий клавиш
def on_press(key):
    global rc_control, is_control

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
        elif key.char == 'y':
            is_control = False
            print('Control disabled')

    except AttributeError:
        # Для специальных клавиш
        print(f'Special key {key} pressed')

def main():
    print("Z/X Thortle \nQ/E Yaw \nW/S Pitch \nA/D Roll")
    global rc_control

    time.sleep(1)

    control.send_RAW_RC([1000, 1000, 1000, 1000, 1000, 1000, 1000])
    control.receive_msg()
    time.sleep(0.5)

    control.send_RAW_RC([100, 1000, 1000, 1000, 2000, 1000, 1000])
    control.receive_msg()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while True:
        if not is_control:
            break

        control.send_RAW_RC(rc_control)
        control.receive_msg()

        #print(f'Current RC control values: {rc_control}')
        time.sleep(0.05)

    listener.stop()

if __name__ == "__main__":
    main()