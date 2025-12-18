
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import time

ip = "127.0.0.1"
port = "1233"

def wall_follow():
    distance = 0.5

    while True:
        distance_to_wall = client.getUltrasonic()
        distance_to_wall /= 100
        print('dist to wall', distance_to_wall)
        pitch_error = distance_to_wall - distance
        # print('pitch_error', pitch_error)

        if distance_to_wall > distance:
            client.setDiod(0, 255, 0)
        else:
            client.setDiod(255, 0, 0)

        r_regulator(pitch_error)


def r_regulator(pitch_error):
    max_speed_roll = 0.2
    pitch_speed = sign(pitch_error) * 0.65
    client.setVelXY(pitch_speed, max_speed_roll) 
    print(pitch_speed, max_speed_roll)

def sign(value):
    if value >= 0:
        value = 0.2
    else:
        value = -0.2
    return value 

client = HighLevelSimClient()

print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")
client.setHeight(0.9)
time.sleep(5)
wall_follow()

