from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient

import time
import math

ip = '127.0.0.1'
port = "1233"


def main():

    client = HighLevelSimClient()
    # подключение
    client.connect(ip, port)
    # взлет
    client.takeoff()
    # устанавливаем целевую высоту
    client.setHeight(1.5) 

    time.sleep(8)
    # обнуляем одометрию
    print(client.setZeroOdomOpticflow())

    # # полет в системе координат одометрии
    # print(client.gotoXYodom(2, 0))

    # print(client.gotoXYodom(2, -2))
    
    # print(client.gotoXYodom(0, -2))
    
    # print(client.gotoXYodom(0, 0))

    # полет в системе координат дрона
    print(client.gotoXYdrone(1, 0))

    print(client.gotoXYdrone(0, 1))
    
    print(client.gotoXYdrone(-1, 0))
    
    print(client.gotoXYdrone(0, -1))

    client.boarding()

    time.sleep(1)

    client.disconnect()


if __name__ == "__main__":
    main()

