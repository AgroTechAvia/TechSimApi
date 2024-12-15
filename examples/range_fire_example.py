from agrotechsimapi import SimClient
import time
import numpy as np

def main():
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        result = client.get_range_data(rangefinder_id = 0, range_min = 0.15, range_max = 10, is_clear = True, range_error = 0.15)
        print(result)
        time.sleep(1/30)

main()