from agrotechsimapi import SimClient
import time
import numpy as np

def main():
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        result = client.get_kinametics_data()
        print('location: ', *result['location'])
        print('orientation: ', *result['orientation'])
        print('linear_velocity: ', *result['location'])
        print('angular_velocity: ', *result['orientation'])
        time.sleep(1/50)

main()