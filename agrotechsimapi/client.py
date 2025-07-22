import msgpackrpc
import cv2
import numpy as np
import random
from enum import Enum

import asyncio
import threading
import grpc
from . import video_pb2
from . import video_pb2_grpc

class CaptureType(Enum):
    color = 0
    thermal = 1
    depth = 2
    spectrum_color = 3 
    spectrum_NIR = 4
    spectrum_SWIR = 5
    spectrum_RE = 6
    spectrum_R = 7
    spectrum_G = 8
    spectrum_B = 9

def post_process(image, gamma=1.0, new_size=(800, 600), saturation=1.0, contrast=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")

    image = cv2.LUT(image, table)

    if new_size is not None:
        image = cv2.resize(image, new_size, interpolation=cv2.INTER_LINEAR)
    
    image = cv2.convertScaleAbs(image, alpha=contrast, beta=0)

    if saturation != 1.0:
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_hsv[:, :, 1] = np.clip(img_hsv[:, :, 1] * saturation, 0, 255).astype(np.uint8)
        image = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

    return image

class VideoStreamSender:
    def __init__(self, camera_id=0, rate=30):
        self.client = SimClient()
        self.camera_id = camera_id
        self.streaming = False
        self.rate = rate

    async def generate_frames(self):
        while self.streaming:
            frame = self.client.get_camera_capture(camera_id=self.camera_id)
            if frame is not None:
                _, buffer = cv2.imencode('.jpg', frame)
                yield video_pb2.Frame(data=buffer.tobytes(), encoding="jpeg")
            await asyncio.sleep(1 / self.rate)

    async def stream(self, port):
        async with grpc.aio.insecure_channel(f"localhost:{port}") as channel:
            stub = video_pb2_grpc.VideoStreamServiceStub(channel)
            await stub.StreamFrames(self.generate_frames())

sender_instance = None
thread_instance = None

class SimClient():
    def __init__(self, 
                 address : str = "127.0.0.1" , 
                 port : int = 8080):
        self.address =  address
        self.port = port
        self.rpc_client = msgpackrpc.Client(msgpackrpc.Address(self.address, self.port), 
                                            timeout = 10, 
                                            pack_encoding = 'utf-8', 
                                            unpack_encoding = 'utf-8')

        self.streaming = False
    
    def __del__(self):
        self.close_connection()

    def close_connection(self):
        if(self.is_connected()):
            self.rpc_client.close()
        
    def add_noise(self,image):
        noise = np.random.normal(0, 1, image.shape).astype(np.uint8)
        noisy_image = cv2.add(image, noise)
        return noisy_image

    def add_artifacts(self,image):
        
 
        h, w, _ = image.shape


        for _ in range(random.randint(1,7)):
            y_line = np.random.randint(0, h)
            width = np.random.randint(3, 10)
            line_end = min(y_line + width, h)

            image[y_line:line_end, :] = np.random.randint(0, 255, size=(line_end - y_line, w, 3), dtype=np.uint8)

        return image

    def is_connected(self):
                result = True
                try:
                    result = self.rpc_client.call('ping')
                except:
                    result = False

                return result

    '''def get_camera_capture(self, camera_id: int = 0, is_clear: bool = True, is_thermal: bool = False, is_depth: bool = False): 

        """
        This function retrieves an image from one of the drone cameras in the simulator. 
        The maximum refresh rate is 20Hz, even if you try to get an image with a higher refresh rate, 
        the camera in the simulator itself is refreshed at 20Hz.
        The image size 640 x 480 (scaled).  

        Args:
            camera_id (int): id of camera
            is_clear(bool) : default True, if False is selected, noise will be generated
            is_thermal(bool) : default False, this flag activate thermal vision
            is_depth(bool) : default False, this flag activate depth vision

        Returns:
            ndarray : openCV image        
        """
        raw_image = self.rpc_client.call('getCameraCapture', camera_id, is_thermal, is_depth)

        if len(raw_image) > 0:
            cv2_image = np.frombuffer(bytes(raw_image), dtype=np.uint8).reshape((360, 480, 4))
            result = post_process(cv2_image, 
                                gamma=1.0, 
                                new_size=(640, 480), 
                                saturation=1.05, 
                                contrast=1)

            if not is_clear:
                result = self.add_noise(result) 
                result = self.add_artifacts(result)  
            return result'''

    def get_laser_scan(self, 
                       angle_min : float = -np.pi/2, 
                       angle_max : float = np.pi/2,
                       range_min : float = 0.1,
                       range_max : float = 30,
                       num_ranges: int = 30,
                       is_clear : bool = False,
                       range_error: float = 0.15):
        
        """
        This function returns a data packet from the rotating lidar on the drone. 
        You can define the angle of view of the lidar (360 degrees by default) by angle_min and angle_max. 
        If the distance is closer or farther than the specified values, the distance will be equal to zero. 
 
        Args:
            angle_min (float): min angle range(degree)
            angle_max (float):  max angle range(degree)
            range_min  (float) : min range for scan distance(meters)
            range_max (float) : max range for scan distance(meters)
            num_ranges (int) : number of traces 
            is_clear (bool) : default True, if False is selected, noise will be generated
            range_error (float) : maximum error variation(if is_clear is false)

        Returns:
            ndarray : distances obtained from lidar scanning(meters)      
        """
        
        laser_scan_data = self.rpc_client.call('getLaserScan', 
                                                   angle_min, 
                                                   angle_max, 
                                                   range_min, 
                                                   range_max, 
                                                   num_ranges)
        
        if(is_clear == False and len(laser_scan_data) == num_ranges):
            noise = np.random.normal(0, range_error, num_ranges)
            laser_scan_data += noise

        
        return laser_scan_data
    
    def get_radar_point(self,
                        radar_id : int = 0,
                        base_angle : float = 45,
                        range_min : float = 0.15,
                        range_max: float = 5,
                        is_clear : bool = True,
                        range_error: float = 0.15,
                        angle_error: float = 0.015):
        
        """
        This function returns information about the nearest point that is within the radar coverage area. 
        The coverage area has the shape of a cone sector. 
        The point information returns in the format of distance and two angles.


        Args:
            radar_id (int) : id of radar
            base_angle (float): cone apex angle
            range_min  (float) : min range for scan distance(meters)
            range_max (float) : max range for scan distance(meters)
            is_clear (bool) : default True, if False is selected, noise will be generated
            range_error (float) : maximum error variation of distance(if is_clear is false)
            angle_error (float) : maximum error variation for angles(if is_clear is false)

        Returns:
            float : point distance(meters)
            float : angle to a point in the horizontal plane(degree)
            float : angle to a point in the vertical plane(degree)
        """
        
        radar_point = self.rpc_client.call('getRadarData',
                                           radar_id,
                                           base_angle,
                                           range_min,
                                           range_max)
        
        radar_point[1] = -radar_point[1]

        if(is_clear == False):
            range_noise = np.random.normal(0,range_error,1)
            radar_point[0] += range_noise

            angle_noise = np.random.normal(0,angle_error,2)
            radar_point[1:] += angle_noise 

        return radar_point
    
    def get_range_data(self,
                        rangefinder_id : int  = 0,
                        range_min : float = 0.15,
                        range_max: float = 10,
                        is_clear : bool = True,
                        range_error: float = 0.15):
        
        """
        This function receives information from the rangefinder

        Args:
            rangefinder_id (int) : id of rangefinder
            range_min  (float) : min range for scan distance(meters)
            range_max (float) : max range for scan distance(meters)
            is_clear (bool) : default True, if False is selected, noise will be generated
            range_error (float) : maximum error variation(if is_clear is false)

        Returns:
            float : point distance(meters)
        """

        range_point = self.rpc_client.call('getRangefinderData', rangefinder_id, range_min, range_max)

        if is_clear == False:
            noise = np.random.normal(0,range_error,1)
            range_point += noise
        
        return range_point

    def set_led_intensity(self,
                            led_id : int = 0,
                            new_intensity : float = 0.5):
        """
        This feature allows you to change the intensity of the brightness of the light diodes on the drone

        Args:
            led_id (int) : id of led diode
            new_intensity  (float) : intensity in range 0..1
        """
        
        self.rpc_client.call('setLedIntensity', led_id, new_intensity)

    def set_led_state(self,
                        led_id : int = 0,
                        new_state : bool = True):
        
        """
        This feature allows you to enable or to disable the light diodes on the drone

        Args:
            led_id (int) : id of led diode
            new_state  (bool) : new diode state
        """
        
        self.rpc_client.call('setLedState', led_id, new_state)

    def get_kinametics_data(self):

        return self.rpc_client.call("getKinematicsData")
    
    def call_event_action(self):
        try:
            return self.rpc_client.call("callEventAction")
        except:
            return False


    def start_streaming(self, port: int, camera_id: int = 0, rate: int = 30):
        global sender_instance, thread_instance

        if sender_instance is not None and sender_instance.streaming:
            print("[INFO] Streaming already running")
            return

        sender_instance = VideoStreamSender(camera_id,rate)
        sender_instance.streaming = True
        

        def run_async():
            asyncio.run(sender_instance.stream(port))

        thread_instance = threading.Thread(target=run_async, daemon=True)
        thread_instance.start()
        print(f"[INFO] Started streaming to port {port}")

    def stop_streaming(self):
        global sender_instance
        if sender_instance:
            sender_instance.streaming = False
            print("[INFO] Stopped streaming")
        else:
            print("[WARN] No active streaming session")

    def get_camera_capture(self, camera_id: int = 0, type: CaptureType = CaptureType.color):

        pp_index = 0
        parameter = 0

        if(type == CaptureType.color):
            pp_index = 0
            parameter = 0
        elif(type == CaptureType.thermal):
            pp_index = 1
            parameter = 0
        elif(type == CaptureType.depth):
            pp_index = 2
            parameter = 0
        elif(type == CaptureType.spectrum_color):
            pp_index = 3
            parameter = 0
        elif(type == CaptureType.spectrum_NIR):
            pp_index = 3
            parameter = 1
        elif(type == CaptureType.spectrum_SWIR):
            pp_index = 3
            parameter = 2
        elif(type == CaptureType.spectrum_RE):
            pp_index = 3
            parameter = 3
        elif(type == CaptureType.spectrum_R):
            pp_index = 3
            parameter = 4
        elif(type == CaptureType.spectrum_G):
            pp_index = 3
            parameter = 5
        elif(type == CaptureType.spectrum_B):
            pp_index = 3
            parameter = 6

        raw_image = self.rpc_client.call('getCameraCapture', camera_id, pp_index, parameter)

        if len(raw_image) > 1:
            cv2_image = np.frombuffer(bytes(raw_image), dtype=np.uint8).reshape((360, 480, 4))
            result = post_process(cv2_image, 
                                gamma=1.0, 
                                new_size=(640, 480), 
                                saturation=1.05, 
                                contrast=1)

            return result
