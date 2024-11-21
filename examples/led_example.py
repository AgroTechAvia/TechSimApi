from agrotechsimapi import SimClient
import time
import numpy as np


#python -m examples.led_example
def main():
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)
    
    intensity = 0
    step = 0.02

    client.set_led_state(0,True)
    while is_loop:

        client.set_led_state(0,False)
        time.sleep(1/2)
        client.set_led_state(0,True)
        time.sleep(1/2)

main()