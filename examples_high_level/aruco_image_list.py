from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import time
import cv2

ip = "127.0.0.1"
port = "1233"

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

# устанавливаем целевую высоту
client.setHeight(1.5)

time.sleep(7)

start_time = time.time() # сохранение текущего времени
duration = 120  # секунд

while True:
    # Проверка времени
    if time.time() - start_time > duration:
        print("Время вышло!")
        break

    error = client.getArucos()
    print(error)
    img = client.getArucosImage()
    cv2.imshow('img', img)
    cv2.waitKey(1)
    time.sleep(0.5)

print(client.boarding())
time.sleep(3)

client.disarmDrone()
time.sleep(1)

client.posholdOff()

client.disconnect()