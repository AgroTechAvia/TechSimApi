from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient

import time
import math

ip = '127.0.0.1'
port = 5762  # TCP-порт MSP; RPC симулятора использует отдельный порт 8080.


def print_drone_position(client):
    """Вывести последнее измеренное положение в координатах карты."""
    kinematics = client.get_sim_kinematics()
    if kinematics is None:
        print("Положение дрона: телеметрия пока недоступна")
        return
    x, y, z = kinematics["location"][:3]
    print(f"Фактическое положение (карта): X={x:.3f}, Y={y:.3f}, Z={z:.3f} м")


def main():

    client = HighLevelSimClient()

     # подключение
    client.connect(ip, port)
    # включаем моторы
    client.armDrone()

    time.sleep(2.0)
    # Enable NAV ALTHOLD after the verified ARM sequence.
    client.altholdOn()

    time.sleep(2.0)
    # взлет
    client.takeoff()
    time.sleep(2)
    client.setHeight(1.35)
   
    time.sleep(8)

    # полет вперед на 2 метра вперед
    client.gotoXYdrone(2, 0)
    print_drone_position(client)

    # поворот на 90 градусов влево
    client.setYaw(-1.57) # еденица измерения в радианах

    client.gotoXYdrone(2, 0)
    print_drone_position(client)

    client.setYaw(3.14)
    
    client.gotoXYdrone(2, 0)
    print_drone_position(client)

    client.setYaw(1.57)
    
    client.gotoXYdrone(2, 0)
    print_drone_position(client)

    client.setYaw(0)

    client.boarding()

    time.sleep(1)

    client.disarmDrone()

    time.sleep(1)

    client.altholdOff()

    client.disconnect()



if __name__ == "__main__":
    main()
