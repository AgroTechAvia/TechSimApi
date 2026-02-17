import time 
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import datetime 

ip = "127.0.0.1" 
port = "1233" 

def WallFollow():
    pid_pitch= PID(0.2, 0.0, 0.5) 
    distance = 0.5 
    pitch_error = 0.0 
    response_distance = 1.0 
    while True: 
        try: 
            distance_to_wall = client.getUltrasonic()
            distance_to_wall /= 100 # переводим сантиметры в метры
            print('distance to wall', distance_to_wall) 
            if distance_to_wall < response_distance: 
                pitch_error = distance_to_wall - distance 
                print('pitch_error', pitch_error) 
                DefaultRegulation(pitch_error, pid_pitch) 
            else: 
                client.setVelXY(0.3, 0) 
            
            if distance_to_wall > 2.0:
                client.setVelXYYaw(0, 0, 0)

        except KeyboardInterrupt: 
            print("KeyboardInterrupt detected, landing the drone...") 
            break 

def DefaultRegulation(pitch_error, pid_pitch: PID): 
    accuracy = 0.1 
    max_pid = 0.2
    max_speed_roll = 0.2
    if abs(pitch_error) > accuracy: 
        pid_pitch.update_control(pitch_error) 
        PID_PITCH = pid_pitch.get_control() 
        PID_PITCH = constrain(PID_PITCH, max_pid) 
    else: 
        PID_PITCH = 0 
    print(f"PID Control: PITCH={PID_PITCH}, ROLL={max_speed_roll}") 
    client.setVelXY(PID_PITCH, max_speed_roll)

def constrain(value, threshold): 
    if value > threshold: 
        value = threshold 
    if value < -threshold: 
        value = -threshold 
    return value 

client = HighLevelSimClient() 

print("connected?", client.connect(ip, port), "\n") 
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n") 
print("takeoff?", client.takeoff(), "\n") 
client.setHeight(0.9) 

time.sleep(5) 
client.setHeight(0.9) 
# time.sleep(5) 
print("Wall Follow: ", WallFollow(), "\n") 
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")