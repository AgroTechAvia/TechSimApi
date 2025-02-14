import agrotechsimapi
import time
import keyboard

IP = "127.0.0.1"
PORT = 41451
isConnected = False
connection_count = 10

client = agrotechsimapi.MultirotorClient(ip = IP, port = PORT)

try:
    client.confirmConnection()
    print("[INFO] Connect successful. Wait start...")
    isConnected = True
except:
    print("[INFO] Connect fail. Please, start drone simulation")
    exit(1)

client.enableApiControl(True)

print("[INFO] Disarming drone. Wait...")
armedDrone = False
client.armDisarm(armedDrone)

client.setMode(agrotechsimapi.AirMultirotorMode.STABILIZE)

print("[INFO] Can flying")


roll = 0
pitch = 0
yaw = 0
throttle = 0.1
roll_control = ['a','d']
pitch_control = ['w', 's']
yaw_control = ['q', 'e']
throttle_control = ['z','x']
arm_disarm = 'f'
to_zero = 't'

print(f"Control keyboard\n\tArm/Disarm: {arm_disarm}, Roll: {roll_control}, Pitch: {pitch_control}, Yaw: {yaw_control}, Throtttle: {throttle_control}, Center: {to_zero}\n")
print(f"Current info\n\tArmed: {armedDrone}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}, Throttle: {throttle}\n")

while isConnected:
    char = keyboard.read_event()

    if char.name == arm_disarm:
        armedDrone = not armedDrone
        print(f"[INFO] Drone arm: {armedDrone}. Wait...")
        client.armDisarm(armedDrone)
        if (armedDrone):
            client.setMode(agrotechsimapi.AirMultirotorMode.POS_ALT_HOLD)
        else:
            client.setMode(agrotechsimapi.AirMultirotorMode.STABILIZE)
            roll = 0
            pitch = 0
            yaw = 0
            throttle = 0
    
    if char.name == pitch_control[0]: pitch -= 0.1
    if char.name == pitch_control[1]: pitch += 0.1

    if char.name == yaw_control[0]: yaw += 0.1
    if char.name == yaw_control[1]: yaw -= 0.1

    if char.name == roll_control[0]: roll -= 0.1
    if char.name == roll_control[1]: roll += 0.1

    if char.name == throttle_control[0]: throttle -= 0.1
    if char.name == throttle_control[1]: throttle += 0.1

    if char.name == to_zero: 
        roll = 0
        pitch = 0
        yaw = 0
        throttle = 0

    if roll > 1: roll = 1
    if roll < -1: roll = -1

    if pitch > 1: pitch = 1
    if pitch < -1: pitch = -1

    if yaw > 1: yaw = 1
    if yaw < -1: yaw = -1

    if throttle > 1: throttle = 1
    if throttle < 0: throttle = 0

    client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, 0.1)
    print(f"Control keyboard\n\tArm/Disarm: {arm_disarm}, Roll: {roll_control}, Pitch: {pitch_control}, Yaw: {yaw_control}, Throtttle: {throttle_control}, Center: {to_zero}\n")
    print(f"Current info\n\tArmed: {armedDrone}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}, Throttle: {throttle}\n")
    time.sleep(0.1)