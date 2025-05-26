import numpy as np
import matplotlib.pyplot as plt
import time
import argparse
from agrotechsimapi import SimClient
import sys

plt.ion() 

def plot_lidar_data(distances):
    angles = np.linspace(-np.pi, np.pi, num=len(distances), endpoint=False)
    
    x = distances * -np.cos(angles + np.pi/2)
    y = distances * np.sin(angles + np.pi/2)
    
    plt.clf()  
    plt.scatter(x, y, s=5)  
    plt.ylim(-12, 12)  
    plt.xlim(-12, 12)  
    plt.title("Lidar Scan Data")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.pause(0.1) 

def cleanup(fig):
    print("\n[Lidar] Shutting down lidar module..")
    plt.close(fig)

def main(args):
    frequency_ = args.frequency
    is_clear_ = True if args.is_clear == 'True' else False

    client = SimClient(address="127.0.0.1", port=8080)
    fig = plt.figure()

    try:
        while True:
            scan = client.get_laser_scan(angle_min=-np.pi, angle_max=np.pi, 
                                       range_max=10, num_ranges=360, 
                                       range_error=0.1, is_clear=is_clear_)
            plot_lidar_data(scan)
            time.sleep(1/frequency_)
            
    except KeyboardInterrupt:
        pass  
    finally:
        cleanup(fig)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=int, default=20)
    parser.add_argument('--is_clear', type=str, default='True')

    args = parser.parse_args()
    main(args)
    