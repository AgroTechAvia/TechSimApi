from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import time

ip = "127.0.0.1"
port = "1233"

# Функция поиска маркера
def search_aruco (yaw_vel):
    client.setVelXYYaw(0, 0, yaw_vel)
    aruco_state = True
    while aruco_state: 
        errors = client.getArucos()
        if errors:
            aruco_state = False
            client.setVelXYYaw(0, 0, 0)

# Функция расчета ошибок
def tracking_aruco():
    pitch_error = 0 
    roll_error = 0
    yaw_error = 0
    distance = 1

    while True:
        errors = client.getArucos()
        if errors:
            distance_to_marker = errors [0]['pose']['position']['z']
            pitch_error = distance_to_marker - distance
            roll_error = errors[0]['pose']['position']['x']
            yaw_error = -errors[0]['pose']['orientation']['z']
            print(f"Дистанция до маркера: {distance_to_marker}")

            aruco_regulation (pitch_error, roll_error, yaw_error)
            
        else:
            aruco_regulation (pitch_error, roll_error, 0)
            print (f"Маркер потерян! Поиск:")
            print (f"pitch_error = {pitch_error}, \
                   roll_error = {roll_error}")

        time.sleep(0.03)

# Выравнивание относительно маркера
def aruco_regulation(pitch_error, roll_error, yaw_error):
    PID_pitch = pitch_error * 0.25
    PID_pitch = constrain (PID_pitch, 0.3)
    PID_roll = roll_error * 0.25
    PID_roll = constrain (PID_roll, 0.3)
    PID_yaw = constrain (yaw_error, 0.4)
    print(f"PID_pitch={PID_pitch}, PID_roll={PID_roll}, \
          PID_yaw={PID_yaw}")
    client.setVelXYYaw(PID_pitch, PID_roll, PID_yaw)

# Функция ограничения величины
def constrain (value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value


client = HighLevelSimClient()

# Подключаемся
client.connect(ip, port)

# Сбрасываем скорости
client.setVelXYYaw(0, 0, 0)

client.armDrone()

time.sleep(2.0)

client.posholdOn()

time.sleep(2.0)

# Взлет
client.takeoff()

client.setHeight(1.5)

time.sleep(7)

# Ищем маркер
search_aruco(0.5)

# Выравнивание относительно маркера
tracking_aruco()
