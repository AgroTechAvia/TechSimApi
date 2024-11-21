from agrotechsimapi import SimClient
import time
import numpy as np
import matplotlib.pyplot as plt
import keyboard

def plot_lidar_data(distances):

    angles = np.linspace(-np.pi, np.pi, num=len(distances), endpoint=False)
    

    x = distances * -np.cos(angles)
    y = distances * np.sin(angles)
    
    plt.clf()  
    plt.scatter(x, y, s=5)  
    plt.ylim(-12, 12)  
    plt.title("Lidar Scan Data")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.pause(0.1) 


#python -m examples.laser_scan_example
def main():
    is_show_plot = False
    is_loop = True
    client = SimClient(address="127.0.0.1", port=8080)

    plt.figure()  

    while is_loop:
        result = client.get_laser_scan(angle_min=-np.pi, angle_max=np.pi, range_max=10, num_ranges=360, range_error=0.1)
        
        print(result)
        if(keyboard.is_pressed("L")):
            is_show_plot = not is_show_plot

        if(is_show_plot):
            plot_lidar_data(result)
        else:
            plt.close()

        time.sleep(1/15)

main()