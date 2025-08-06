from agrotechsimapi import SimClient
import time
import numpy as np
import matplotlib.pyplot as plt

is_loop = True

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

def main():
    client = SimClient(address="127.0.0.1", port=8080)

    plt.figure()      
    while is_loop:
        result = client.get_laser_scan(angle_min=-np.pi, angle_max=np.pi, range_max=10, num_ranges=360, range_error=0.1, is_clear=True)
        plot_lidar_data(result)

        time.sleep(1/15)
    

main()