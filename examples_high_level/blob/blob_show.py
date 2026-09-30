from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient

import cv2

import time
import math

ip = '127.0.0.1'
port = 5762


BLOBS_DICT = {0: "Красный", 1: "Зеленый", 2: "Синий"}


def main():

    client = HighLevelSimClient()

    client.connect(ip, port)
    
    client.armDrone()

    time.sleep(2.0)

    client.altholdOn()

    time.sleep(2.0)

    client.takeoff()   

    time.sleep(3)

    try:
        print(f"Начало поиска блобов")
        client.setVelXYYaw(0, 0, 0.7)
        while True:
            blobs = client.getBlobs()
            image = client.getBlobsImage()
            if image is not None:
                cv2.imshow("blob", image)
                cv2.waitKey(1)
            print(f"Количество блобов на картинке: {len(blobs)}")
            i = 0
            for blob in blobs:
                id = int(blob["id"])
                blob_color = BLOBS_DICT[id]
                print(f"Наблюдаю маркер номер {i} с цветом: {blob_color}")
                i += 1
                # break
            print("-" * 20)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        client.setVelXYYaw(0, 0, 0)
        print(f"Остановка поиска блобов")
    finally:
        client.boarding()
        client.disconnect()
        cv2.destroyAllWindows()





if __name__ == "__main__":
    main()

