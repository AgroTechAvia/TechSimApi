from agrotechsimapi import SimClient
import time
import numpy as np
import argparse

def main(args):
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        result = client.get_range_data(rangefinder_id = args.range_fire_num, range_min = 0.15, range_max = 10, is_clear = True, range_error = 0.15)
        print(result)
        time.sleep(1/30)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--range_fire_num', type=int, help='Range fire number: 0(front)/1(back)/2(bottom)', default=0)
    args = parser.parse_args()
    main(args)