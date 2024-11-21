from agrotechsimapi import SimClient
import time
import numpy as np

#python -m examples.range_fire_example
def main():
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    while is_loop:
        #result = client.get_range_data(0, 150, 2000)
        result = client.get_radar_point(0,45, 150, 2000,True)
        print(result)
        time.sleep(1/30)

main()