import time
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import datetime
import math
import cv2

ip = "127.0.0.1"
port = "1233"

last_time = None

client = HighLevelSimClient()

print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
client.takeoff()
# client.setHeight(1.5)
time.sleep(8)

while True:
    # # для дальномера
    # print(client.getUltrasonic())
    # time.sleep(0.5)

    # # для блобов
    # error = client.getBlobs()
    # image = client.getBlobsImage()
    
    # для ArUco-маркеров
    error = client.getArucos()
    image = client.getArucosImage()

    # # вывод изображения
    cv2.imshow("image", image)
    cv2.waitKey(1)

    if error:

        # # вывод информации о блобах 
        # camera_center_x = 320/2
        # camera_center_y = 240/2
        # target_area = 2300
        # blob_area = error[0]["size"]["x"] * error[0]["size"]["y"]
        # pitch_error = target_area - blob_area
        # roll_error = error[0]["center"]["x"] - camera_center_x
        # height_error = error[0]["center"]["y"] - camera_center_y
        # print(error)
        # print("Площадь блоба:", blob_area)
        # print(f"Pitch_error: {pitch_error}, Roll_error: {roll_error}, Height_error: {height_error}")

        # вывод информации о ArUco-маркерах
        distance = 1
        distance_to_marker = error[0]['pose']['position']['z']
        pitch_error = distance_to_marker - distance
        roll_error = error[0]['pose']['position']['x']
        yaw_error = -1 * error[0]['pose']['orientation']['z']
        print(error)
        print(f"Pitch_error: {pitch_error}, Roll_error: {roll_error}, Yaw_error: {yaw_error}")

    else:
        continue
    time.sleep(0.5)


# print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
# print("boarding?", client.boarding(), "\n")
# time.sleep(2.0)
# client.disconnect()