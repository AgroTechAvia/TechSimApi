from agrotechsimapi import SimClient
import time
import numpy as np
import math

def main():
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        result = client.get_kinametics_data()
        print('location: ', *result['location'])
        print('orientation: ', *result['orientation'])
        print('angular_velocity: ', *result['angular_velocity'])
        print('linear_velocity: ', *result['linear_velocity'])
        time.sleep(1/30)

main()