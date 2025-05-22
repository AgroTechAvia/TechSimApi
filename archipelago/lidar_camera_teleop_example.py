import subprocess
import argparse
import sys
    
def main(args):
    lidar_process = subprocess.Popen([sys.executable, "archipelago\modules\lidar_driver.py", 
    "--frequency", str(args.lidar_frequency), 
    "--is_clear", str(args.lidar_clear)])

    camera_process = subprocess.Popen([sys.executable, "archipelago\modules\camera_driver.py", 
    "--frequency", str(args.camera_frequency)])
    
    teleop_process =  subprocess.Popen([sys.executable, "archipelago\modules\input_driver.py", 
    "--frequency", str(args.camera_frequency), 
    "--is_action", "False",
    "--inav_host", args.inav_host,
    "--inav_port", str(args.inav_port)])

    lidar_process.wait()
    camera_process.wait()
    teleop_process.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--lidar_frequency', type=int, default=20)
    parser.add_argument('--lidar_clear', type=str, default="False")

    parser.add_argument('--camera_frequency', type=int, default=24)

    parser.add_argument('--teleop_frequency', type=int, default=24)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    main(args)