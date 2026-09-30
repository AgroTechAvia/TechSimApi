from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import time
import cv2

ip = "127.0.0.1"
port = 5762


def main():
    client = HighLevelSimClient()
    client.connect(ip, port)
    client.setVelXYYaw(0, 0, 0)
    client.armDrone()
    time.sleep(2.0)
    client.altholdOn()
    time.sleep(2.0)
    client.takeoff()
    client.setHeight(1.5)
    time.sleep(7)

    start_time = time.time()
    duration = 120
    try:
        while time.time() - start_time <= duration:
            markers = client.getArucos()
            print(markers)
            image = client.getArucosImage()
            if image is not None:
                cv2.imshow("img", image)
                cv2.waitKey(1)
            time.sleep(0.5)
    finally:
        client.boarding()
        client.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
