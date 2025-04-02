from agrotechsimapi import SimClient
import time
import numpy as np
import argparse

def main(args):
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        result = client.get_radar_point(radar_id=args.radar_num,base_angle=45, range_min=150, range_max=2000,is_clear=True,range_error=0.15,angle_error=0.015)
        
        print(result)
        time.sleep(1/30)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--radar_num', type=int, help='Radar number: 0(front)/1(back)/2(bottom)', default=0)
    args = parser.parse_args()
    main(args)