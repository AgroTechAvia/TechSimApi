import time
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import datetime
import math
import cv2

ip = "127.0.0.1"
port = 5762

last_time = None

def main():
    client = HighLevelSimClient()
    client.connect(ip, port)
    client.setVelXYYaw(0, 0, 0)
    client.armDrone()
    client.takeoff()
    time.sleep(8)
    try:
        while True:
            markers = client.getArucos()
            image = client.getArucosImage()
            if image is not None:
                cv2.imshow("image", image)
                cv2.waitKey(1)

            if markers:
                distance_to_marker = markers[0]["pose"]["position"]["z"]
                pitch_error = distance_to_marker - 1.0
                roll_error = markers[0]["pose"]["position"]["x"]
                yaw_error = -markers[0]["pose"]["orientation"]["z"]
                print(markers)
                print(
                    f"Pitch_error: {pitch_error}, Roll_error: {roll_error}, "
                    f"Yaw_error: {yaw_error}"
                )
            time.sleep(0.5)
    finally:
        client.boarding()
        client.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
