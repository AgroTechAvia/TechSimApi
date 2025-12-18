import time
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import datetime
import math
import cv2

ip = "127.0.0.1"
port = "1233"

last_time = None

def BlobRegulation(target_area):
    # Настройка PID регулятора для компоненты ROLL(в данном случае P-регулятор)
    pid_pitch = PID(0.0025, 0.002, 0.02)
    pid_roll = PID(0.0025, 0.002, 0.02)
    camera_center_x = 640/2
    camera_center_y = 480/2
    blob_area = 0
    accuracy_area = 50
    accuracy = 20
    flag = False

    while True:
        error = client.getBlobs()
        if error:
                image = client.getBlobsImage()
                cv2.imshow("blob", image)
                cv2.waitKey(1)
                flag = True
                blob_area = error[0]["size"]["x"] * error[0]["size"]["y"]
                pitch_error = target_area - blob_area
                roll_error = error[0]["center"]["x"] - camera_center_x
                height_error = error[0]["center"]["y"] - camera_center_y

                if abs(pitch_error)<=accuracy_area and abs(roll_error)<=accuracy and abs(height_error)<=accuracy:
                    client.setVelXYYaw(0, 0, 0)
                    print('waiting for 5 second')
                    break
                else:
                    blob_Regulation(pitch_error, roll_error, pid_pitch, pid_roll)
                    height_regulation(height_error)
            
        else:
            client.setVelXYYaw(0.3, 0.0, 0.0)
            print('я слеп')
            
def blob_Regulation(pitch_error, roll_error, pid_pitch:PID, pid_roll:PID):
    pid_pitch.update_control(pitch_error)
    PID_PITCH = pid_pitch.get_control()
    PID_PITCH = constrain(PID_PITCH, 0.3)
    pid_roll.update_control(roll_error)
    PID_ROLL = pid_roll.get_control()
    PID_ROLL = constrain(PID_ROLL, 0.3)
    print(f'pitch_error: {pitch_error}, roll_error: {roll_error}')
    print(f"PID: {PID_PITCH, PID_ROLL}")
    client.setVelXYYaw(PID_PITCH, PID_ROLL, 0)
    # client.setVelXYYaw(0.0, 0.0, 0.0)

def height_regulation(height_error):
    height = client.getHeightRange()
    height_error *= 0.01
    new_height = height - height_error
    print(f'New_height {new_height}')
    client.setHeight(new_height)

# Функция для ограничения значения переменной
def constrain(value, threshold):
   if value > threshold:
       value = threshold
   if value < -threshold:
       value = -threshold
   return value       

client = HighLevelSimClient()

print("connected?", client.connect(ip, port), "\n")
time.sleep(2)
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
client.armDrone()
time.sleep(2.0)
    # включаем пежим удержания позиции
client.posholdOn()

time.sleep(2.0)
client.takeoff()
time.sleep(7)
# client.setYaw(1.57)
BlobRegulation(2300)
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")
client.posholdOff()
time.sleep(2.0)
client.disarmDrone()
time.sleep(2.0)
client.disconnect()