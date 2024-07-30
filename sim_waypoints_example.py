import agrotechsimapi
import cv2
import numpy as np
import os
import time
import tempfile

print("[INFO] Connect to simulator...")

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

#activation agrodrone
#client.setMode(agrotechsimapi.AirMultirotorMode.PLANE)

client.armDisarm(True)

client.addWaypointsToMission([
    agrotechsimapi.Waypoint(1, 1,-35.3632238,149.1651379, 200, 0, 0, 0, 0),
    agrotechsimapi.Waypoint(2, 1,-35.3632238,149.1651379, 200, 0, 0, 0, 0),
    agrotechsimapi.Waypoint(3,1, -35.3615439,149.1644996, 200, 0, 0, 0, 0),
])

client.setMode(agrotechsimapi.AirMultirotorMode.WP_MODE)
input()

client.armDisarm(False)
client.enableApiControl(False)
