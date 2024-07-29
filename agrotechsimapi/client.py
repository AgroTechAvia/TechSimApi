from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import time
import math
import logging

class Waypoint(MsgpackMixin):
    wp_no = 0.0
    action = 0.0
    lat = 0.0
    lon = 0.0
    alt = 0.0
    p1 = 0.0
    p2 = 0.0
    p3 = 0.0
    flag = 0.0
    
    def __init__(self, wp_no, action, lat, lon, alt, p1, p2, p3, flag) -> None:
        super().__init__()
        self.wp_no = wp_no
        self.action = action
        self.lat = int(lat * 10000000)
        self.lon = int(lon * 10000000)
        self.alt = int(alt * 100)
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.flag = flag

class VehicleClient:
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')

#----------------------------------- Common vehicle APIs ---------------------------------------------
    def reset(self):
        """
        Reset the vehicle to its original starting state

        Note that you must call `enableApiControl` and `armDisarm` again after the call to reset
        """
        self.client.call('reset')

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout

        Returns:
            bool:
        """
        return self.client.call('ping')
    
    def getClientVersion(self):
        return 1 # sync with C++ client

    def getServerVersion(self):
        return self.client.call('getServerVersion')

    def getMinRequiredServerVersion(self):
        return 1 # sync with C++ client

    def getMinRequiredClientVersion(self):
        return self.client.call('getMinRequiredClientVersion')



    def enableApiControl(self, is_enabled, vehicle_name = ''):
        """
        Enables or disables API control for vehicle corresponding to vehicle_name

        Args:
            is_enabled (bool): True to enable, False to disable API control
            vehicle_name (str, optional): Name of the vehicle to send this command to
        """
        self.client.call('enableApiControl', is_enabled, vehicle_name)

    def isApiControlEnabled(self, vehicle_name = ''):
        """
        Returns true if API control is established.

        If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, `isApiControlEnabled` should return true.

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            bool: If API control is enabled
        """
        res = self.client.call('isApiControlEnabled', vehicle_name)
        time.sleep(3)
        return res
    
    def armDisarm(self, arm, vehicle_name = ''):
        """
        Arms or disarms vehicle

        Args:
            arm (bool): True to arm, False to disarm the vehicle
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            bool: Success
        """
        res = self.client.call('armDisarm', arm, vehicle_name)
        time.sleep(3)
        return res


    def simPause(self, is_paused):
        """
        Pauses simulation

        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        self.client.call('simPause', is_paused)

    def simIsPause(self):
        """
        Returns true if the simulation is paused

        Returns:
            bool: If the simulation is paused
        """
        return self.client.call("simIsPaused")

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        """
        if self.ping():
            print("Connected!")
        else:
             print("Ping returned false!")
        server_ver = self.getServerVersion()
        client_ver = self.getClientVersion()
        server_min_ver = self.getMinRequiredServerVersion()
        client_min_ver = self.getMinRequiredClientVersion()

        ver_info = "Client Ver:" + str(client_ver) + " (Min Req: " + str(client_min_ver) + \
              "), Server Ver:" + str(server_ver) + " (Min Req: " + str(server_min_ver) + ")"

        if server_ver < server_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim server is of older version and not supported by this client. Please upgrade!")
        elif client_ver < client_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim client is of older version and not supported by this server. Please upgrade!")
        else:
            print(ver_info)
        print('')

    def simGetImage(self, camera_name, image_type, vehicle_name = '', external = False):
        """
        Get a single image

        Returns bytes of png format image which can be dumped into abinary file to create .png image
        `string_to_uint8_array()` can be used to convert into Numpy unit8 array
        See https://microsoft.github.io/AirSim/image_apis/ for details

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Name of the vehicle with the camera
            external (bool, optional): Whether the camera is an External Camera

        Returns:
            Binary string literal of compressed png image
        """
        camera_name = str(camera_name)

        result = self.client.call('simGetImage', camera_name, image_type, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

    def simGetImages(self, requests, vehicle_name = '', external = False):
        """
        Get multiple images

        See https://microsoft.github.io/AirSim/image_apis/ for details and examples

        Args:
            requests (list[ImageRequest]): Images required
            vehicle_name (str, optional): Name of vehicle associated with the camera
            external (bool, optional): Whether the camera is an External Camera

        Returns:
            list[ImageResponse]:
        """
        responses_raw = self.client.call('simGetImages', requests, vehicle_name, external)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    def simGetFocalLength(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocalLength", camera_name, vehicle_name, external)

    def simSetFocalLength(self, focal_length, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocalLength", focal_length, camera_name, vehicle_name, external)

    def simEnableManualFocus(self, enable, camera_name, vehicle_name = '', external = False):  
        self.client.call("simEnableManualFocus", enable, camera_name, vehicle_name, external)

    def simGetFocusDistance(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocusDistance", camera_name, vehicle_name, external)

    def simSetFocusDistance(self, focus_distance, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocusDistance", focus_distance, camera_name, vehicle_name, external)

    def simGetFocusAperture(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocusAperture", camera_name, vehicle_name, external)

    def simSetFocusAperture(self, focus_aperture, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocusAperture", focus_aperture, camera_name, vehicle_name, external)

    def simEnableFocusPlane(self, enable, camera_name, vehicle_name = '', external = False):  
        self.client.call("simEnableFocusPlane", enable, camera_name, vehicle_name, external)

    def simGetCurrentFieldOfView(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetCurrentFieldOfView", camera_name, vehicle_name, external)

    def getDistanceSensorData(self, distance_sensor_name = '', vehicle_name = ''):
        """
        Args:
            distance_sensor_name (str, optional): Name of Distance Sensor to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            DistanceSensorData:
        """
        return DistanceSensorData.from_msgpack(self.client.call('getDistanceSensorData', distance_sensor_name, vehicle_name))

    def getLidarData(self, lidar_name = '', vehicle_name = ''):
        """
        Args:
            lidar_name (str, optional): Name of Lidar to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            LidarData:
        """
        return LidarData.from_msgpack(self.client.call('getLidarData', lidar_name, vehicle_name))

    def simGetLidarSegmentation(self, lidar_name = '', vehicle_name = ''):
        """
        NOTE: Deprecated API, use `getLidarData()` API instead
        Returns Segmentation ID of each point's collided object in the last Lidar update

        Args:
            lidar_name (str, optional): Name of Lidar sensor
            vehicle_name (str, optional): Name of the vehicle wth the sensor

        Returns:
            list[int]: Segmentation IDs of the objects
        """
        logging.warning("simGetLidarSegmentation API is deprecated, use getLidarData() API instead")
        return self.getLidarData(lidar_name, vehicle_name).segmentation
    
    def getImuData(self, imu_name = '', vehicle_name = ''):
        """
        Args:
            imu_name (str, optional): Name of IMU to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            ImuData:
        """
        return ImuData.from_msgpack(self.client.call('getImuData', imu_name, vehicle_name))

    def getBarometerData(self, barometer_name = '', vehicle_name = ''):
        """
        Args:
            barometer_name (str, optional): Name of Barometer to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            BarometerData:
        """
        return BarometerData.from_msgpack(self.client.call('getBarometerData', barometer_name, vehicle_name))


#----------------------------------- Multirotor APIs ---------------------------------------------
class MultirotorClient(VehicleClient, object):
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        super(MultirotorClient, self).__init__(ip, port, timeout_value)
   
    def addWaypointsToMission(self, waypoints: list[Waypoint], vehicle_name=''):
        self.client.call('addWaypointsToMission', waypoints, vehicle_name)
        time.sleep(1)
        return   

    def setMode(self, mode, vehicle_name=''):
        return self.client.call_async('setMode', mode.value, vehicle_name)
    
    def setLEDCmd(self, id_: int, flag: bool, vehicle_name=''):
        self.client.call('setLEDcmd', id_, flag,  vehicle_name)
        return   

    def moveByRollPitchYawThrottleAsync(self, roll, pitch, yaw, throttle, duration, vehicle_name = ''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw angle are given in **degrees** when using PX4 and in **radians** when using SimpleFlight, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.

        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.

            - Y axis is along the **Left** direction of the quadrotor.

            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.

            - Z axis is along the **Up** direction.

            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.

        Args:
            roll (float): Desired roll angle.
            pitch (float): Desired pitch angle.
            yaw (float): Desired yaw angle.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByRollPitchYawThrottle', roll, -pitch, -yaw, throttle, duration, vehicle_name)

#query vehicle state
    def getMultirotorState(self, vehicle_name = ''):
        """
        The position inside the returned MultirotorState is in the frame of the vehicle's starting point

        Args:
            vehicle_name (str, optional): Vehicle to get the state of

        Returns:
            MultirotorState:
        """
        return MultirotorState.from_msgpack(self.client.call('getMultirotorState', vehicle_name))
    getMultirotorState.__annotations__ = {'return': MultirotorState}
#query rotor states
    def getRotorStates(self, vehicle_name = ''):
        """
        Used to obtain the current state of all a multirotor's rotors. The state includes the speeds,
        thrusts and torques for all rotors.

        Args:
            vehicle_name (str, optional): Vehicle to get the rotor state of

        Returns:
            RotorStates: Containing a timestamp and the speed, thrust and torque of all rotors.
        """
        return RotorStates.from_msgpack(self.client.call('getRotorStates', vehicle_name))
    getRotorStates.__annotations__ = {'return': RotorStates}

#----------------------------------- Car APIs ---------------------------------------------
class CarClient(VehicleClient, object):
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        super(CarClient, self).__init__(ip, port, timeout_value)

    def setCarControls(self, controls, vehicle_name = ''):
        """
        Control the car using throttle, steering, brake, etc.

        Args:
            controls (CarControls): Struct containing control values
            vehicle_name (str, optional): Name of vehicle to be controlled
        """
        self.client.call('setCarControls', controls, vehicle_name)

    def getCarState(self, vehicle_name = ''):
        """
        The position inside the returned CarState is in the frame of the vehicle's starting point

        Args:
            vehicle_name (str, optional): Name of vehicle

        Returns:
            CarState:
        """
        state_raw = self.client.call('getCarState', vehicle_name)
        return CarState.from_msgpack(state_raw)

    def getCarControls(self, vehicle_name=''):
        """
        Args:
            vehicle_name (str, optional): Name of vehicle

        Returns:
            CarControls:
        """
        controls_raw = self.client.call('getCarControls', vehicle_name)
        return CarControls.from_msgpack(controls_raw)
    
    def startWPMission(self, vehicle_name=''):
        res = self.client.call('startWPMission', vehicle_name)
        time.sleep(1)
        return res
    
    def addWaypointsToMission(self, waypoints: list[Waypoint], vehicle_name=''):
        self.client.call('addWaypointsToMission', waypoints, vehicle_name)
        time.sleep(1)
        return    

