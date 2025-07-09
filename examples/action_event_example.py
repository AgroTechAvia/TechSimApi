from agrotechsimapi import SimClient
from pynput import keyboard
import time

client = None
listener = None
is_run = True
def on_i_press(key):
    global client,listener,is_run

    if key.char == 'i':
        print("Call event action")
        client.call_event_action()
    elif key.char == 'o':
        is_run = False
    else:
        pass

def main():
    global client,listener,is_run

    client = SimClient(address = "127.0.0.1", port = 8080)
    listener = keyboard.Listener(on_press=on_i_press)
    listener.start()

    while is_run:
        time.sleep(1)
    listener.stop()
if __name__ == "__main__":

    main()