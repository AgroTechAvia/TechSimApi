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
    print("[INFO] Connect successful")
    isConnected = True
except:
    print("[INFO] Connect fail")

client.enableApiControl(True)

print("[INFO] API Control enabled: %s" % client.isApiControlEnabled())


print("[INFO] Arm drone")
client.armDisarm(True)

client.setMode(agrotechsimapi.AirMultirotorMode.POS_ALT_HOLD)

roll = 0
pitch = 0
yaw = 0
throttle = 0.1


while isConnected:
    char = keyboard.read_event()
    
    if char.name == 'w': pitch += 0.1
    if char.name == 's': pitch -= 0.1

    if char.name == 'a': yaw += 0.1
    if char.name == 'd': yaw -= 0.1

    if char.name == 'q': roll += 0.1
    if char.name == 'e': roll -= 0.1

    if char.name == 'z': throttle += 0.1
    if char.name == 'x': throttle -= 0.1

    if char.name == 't': 
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
    print("roll: ",roll, " pitch: ",pitch, " yaw: ",yaw, " throttle: ",throttle)
    time.sleep(0.1)