import subprocess
import argparse
import sys
from pathlib import Path

def main(args):

    camera_path = Path("archipelago","modules","camera_driver.py")
    input_path = Path("archipelago","modules","input_driver.py")
    
    camera_process = subprocess.Popen([sys.executable, str(camera_path), 
                                       "--frequency", str(args.camera_frequency)])
    
    teleop_process = subprocess.Popen([sys.executable, str(input_path), 
                                       "--frequency", str(args.camera_frequency), 
                                       "--is_action", "False",
                                       "--inav_host", args.inav_host,
                                       "--inav_port", str(args.inav_port)])

    camera_process.wait()
    teleop_process.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--camera_frequency', type=int, default=24)

    parser.add_argument('--teleop_frequency', type=int, default=24)
    parser.add_argument('--inav_host', type=str, default='127.0.0.1')
    parser.add_argument('--inav_port', type=int, default=5762)

    args = parser.parse_args()
    main(args)