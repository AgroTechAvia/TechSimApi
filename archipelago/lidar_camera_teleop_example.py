import subprocess
import argparse
import sys
from pathlib import Path
import time

def main(args):
    processes = []  
    
    try:
        lidar_path = Path("archipelago","modules","lidar_driver.py")
        camera_path = Path("archipelago","modules","camera_driver.py")
        input_path = Path("archipelago","modules","input_driver.py")
        

        lidar_process = subprocess.Popen([sys.executable, str(lidar_path), 
                                        "--frequency", str(args.lidar_frequency), 
                                        "--is_clear", str(args.lidar_clear)])
        processes.append(lidar_process)

        camera_process = subprocess.Popen([sys.executable, str(camera_path), 
                                        "--frequency", str(args.camera_frequency),
                                        "--camera_num", str(args.camera_num)])
        processes.append(camera_process)
        
        teleop_process = subprocess.Popen([sys.executable, str(input_path), 
                                        "--frequency", str(args.camera_frequency), 
                                        "--is_action", "False",
                                        "--inav_host", args.inav_host,
                                        "--inav_port", str(args.inav_port)])
        processes.append(teleop_process)


        while True:
            time.sleep(1)

            for p in processes[:]:  
                if p.poll() is not None:  
                    processes.remove(p)
            if not processes:  
                break
                
    except KeyboardInterrupt:
        print("\n[Main Programm] Received Ctrl+C signal, terminating processes...")
        for p in processes:
            p.terminate()  
        for p in processes:
            p.wait()       
        print("[Main Programm] All processes are complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--lidar_frequency', type=int, default=20)
    parser.add_argument('--lidar_clear', type=str, default="False")

    parser.add_argument('--camera_frequency', type=int, default=24)
    parser.add_argument('--camera_num', type=int, default=0)

    parser.add_argument('--teleop_frequency', type=int, default=24)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    main(args)