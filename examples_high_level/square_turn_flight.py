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
    # включаем моторы
    client.armDrone()

    time.sleep(2.0)
    # включаем пежим удержания позиции
    client.posholdOn()

    time.sleep(2.0)
    # взлет
    client.takeoff()
   
    time.sleep(8)

    # полет вперед на 2 метра вперед
    client.gotoXYdrone(2, 0)

    # поворот на 90 градусов влево
    client.setYaw(-1.57) # еденица измерения в радианах

    client.gotoXYdrone(2, 0)

    client.setYaw(3.14)
    
    client.gotoXYdrone(2, 0)

    client.setYaw(1.57)
    
    client.gotoXYdrone(2, 0)

    client.setYaw(0)

    client.boarding()

    time.sleep(1)

    client.disarmDrone()

    time.sleep(1)

    client.posholdOff()

    client.disconnect()



if __name__ == "__main__":
    main()

