import msgpackrpc
import cv2
import numpy as np
import random
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping, Tuple

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


@dataclass(frozen=True)
class PointCloud:
    """One organized 3D lidar frame in the sensor-local ROS FLU coordinate system.

    ``points`` has shape ``(channel_count, points_per_channel, 3)`` and stores
    metres. Invalid returns contain NaN coordinates and are marked by ``valid``.
    The binary layout metadata mirrors ``sensor_msgs/msg/PointCloud2`` so that
    ROS 2 adapters can reuse ``data``, ``fields``, ``point_step`` and
    ``row_step`` without coordinate conversion or repacking. The mounting
    orientation is an ``(x, y, z, w)`` quaternion from lidar frame to body frame.
    """

    points: np.ndarray
    intensity: np.ndarray
    ring: np.ndarray
    return_type: np.ndarray
    valid: np.ndarray
    time_offset_ns: np.ndarray
    schema_version: int
    sensor_id: int
    sensor_position_body: np.ndarray
    sensor_orientation_body: np.ndarray
    timestamp_ns: int
    sequence: int
    frame_id: str
    scan_duration_ns: int
    fields: Tuple[Mapping[str, Any], ...]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool
    valid_point_count: int
    acquisition_deadline_missed: bool

    @property
    def channel_count(self) -> int:
        return int(self.points.shape[0])

    @property
    def points_per_channel(self) -> int:
        return int(self.points.shape[1])

    @property
    def valid_points(self) -> np.ndarray:
        """Return valid XYZ points flattened to ``(N, 3)``."""
        return self.points[self.valid]


_POINT_CLOUD_FIELD_LAYOUT = {
    "x": (0, 7),
    "y": (4, 7),
    "z": (8, 7),
    "intensity": (12, 7),
    "ring": (16, 4),
    "return_type": (18, 2),
    "valid": (19, 2),
    "time_offset_ns": (20, 6),
}

_POINT_CLOUD_RESPONSE_KEYS = (
    "success", "error", "schema_version", "sensor_id", "sequence", "frame_id",
    "stamp_sec", "stamp_nanosec", "scan_duration_ns", "height", "width", "fields",
    "is_bigendian", "point_step", "row_step", "data", "is_dense",
    "valid_point_count", "acquisition_deadline_missed",
)
_POINT_CLOUD_RESPONSE_KEYS_V2 = _POINT_CLOUD_RESPONSE_KEYS + (
    "sensor_position_body", "sensor_orientation_body",
)
_POINT_CLOUD_FIELD_KEYS = ("name", "offset", "datatype", "count")


def _string_key_mapping(value, sequence_keys=None):
    if sequence_keys is not None and isinstance(value, (list, tuple)):
        if len(value) != len(sequence_keys):
            raise RuntimeError("Incompatible simulator RPC response array length")
        value = dict(zip(sequence_keys, value))
    if not isinstance(value, Mapping):
        raise RuntimeError(
            "Incompatible simulator RPC response: point cloud must be a map"
        )
    return {
        key.decode("utf-8") if isinstance(key, bytes) else key: item
        for key, item in value.items()
    }


def _decode_point_cloud(response) -> PointCloud:
    sequence_keys = _POINT_CLOUD_RESPONSE_KEYS
    if isinstance(response, (list, tuple)) and len(response) == len(_POINT_CLOUD_RESPONSE_KEYS_V2):
        sequence_keys = _POINT_CLOUD_RESPONSE_KEYS_V2
    response = _string_key_mapping(response, sequence_keys)
    if not response.get("success", False):
        error = response.get("error", "unknown simulator error")
        if isinstance(error, bytes):
            error = error.decode("utf-8", errors="replace")
        raise RuntimeError(f"3D lidar scan failed: {error}")

    schema_version = int(response.get("schema_version", -1))
    if schema_version not in (1, 2):
        raise RuntimeError(
            "Unsupported 3D lidar point-cloud schema version: "
            f"{response.get('schema_version')!r}"
        )

    height = int(response["height"])
    width = int(response["width"])
    point_step = int(response["point_step"])
    row_step = int(response["row_step"])
    is_bigendian = bool(response["is_bigendian"])
    if height <= 0 or width <= 0 or point_step != 24 or row_step < width * point_step:
        raise RuntimeError("Invalid 3D lidar point-cloud dimensions or strides")

    fields = tuple(
        _string_key_mapping(field, _POINT_CLOUD_FIELD_KEYS)
        for field in response["fields"]
    )
    fields = tuple({
        **field,
        "name": field["name"].decode("utf-8")
        if isinstance(field["name"], bytes) else field["name"],
    } for field in fields)
    actual_layout = {
        str(field["name"]): (int(field["offset"]), int(field["datatype"]))
        for field in fields
    }
    if any(actual_layout.get(name) != layout for name, layout in _POINT_CLOUD_FIELD_LAYOUT.items()):
        raise RuntimeError("Incompatible 3D lidar PointCloud2 field layout")

    raw_data = bytes(response["data"])
    required_size = height * row_step
    if len(raw_data) < required_size:
        raise RuntimeError(
            f"Truncated 3D lidar payload: expected {required_size} bytes, got {len(raw_data)}"
        )

    byte_order = ">" if is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z", "intensity", "ring", "return_type", "valid", "time_offset_ns"],
        "formats": [
            byte_order + "f4", byte_order + "f4", byte_order + "f4", byte_order + "f4",
            byte_order + "u2", "u1", "u1", byte_order + "u4",
        ],
        "offsets": [0, 4, 8, 12, 16, 18, 19, 20],
        "itemsize": point_step,
    })
    records = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=raw_data,
        strides=(row_step, point_step),
    )
    points = np.stack((records["x"], records["y"], records["z"]), axis=-1)
    frame_id = response["frame_id"]
    if isinstance(frame_id, bytes):
        frame_id = frame_id.decode("utf-8")
    sensor_position_body = np.asarray(
        response.get("sensor_position_body", (0.0, 0.0, 0.0)), dtype=np.float64
    )
    sensor_orientation_body = np.asarray(
        response.get("sensor_orientation_body", (0.0, 0.0, 0.0, 1.0)), dtype=np.float64
    )
    if sensor_position_body.shape != (3,) or sensor_orientation_body.shape != (4,):
        raise RuntimeError("Invalid 3D lidar mounting transform")
    orientation_norm = np.linalg.norm(sensor_orientation_body)
    if not np.all(np.isfinite(sensor_position_body)) or not np.isfinite(orientation_norm) or orientation_norm == 0.0:
        raise RuntimeError("Non-finite 3D lidar mounting transform")
    sensor_orientation_body = sensor_orientation_body / orientation_norm

    return PointCloud(
        points=points,
        intensity=records["intensity"],
        ring=records["ring"],
        return_type=records["return_type"],
        valid=records["valid"].astype(bool, copy=False),
        time_offset_ns=records["time_offset_ns"],
        schema_version=schema_version,
        sensor_id=int(response["sensor_id"]),
        sensor_position_body=sensor_position_body,
        sensor_orientation_body=sensor_orientation_body,
        timestamp_ns=int(response["stamp_sec"]) * 1_000_000_000 + int(response["stamp_nanosec"]),
        sequence=int(response["sequence"]),
        frame_id=str(frame_id),
        scan_duration_ns=int(response["scan_duration_ns"]),
        fields=fields,
        is_bigendian=is_bigendian,
        point_step=point_step,
        row_step=row_step,
        data=raw_data,
        is_dense=bool(response["is_dense"]),
        valid_point_count=int(response["valid_point_count"]),
        acquisition_deadline_missed=bool(response["acquisition_deadline_missed"]),
    )

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
        self.drone_name = None
    
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

    def get_drone_name(self):
        """Get the simulator owner name and cache it in ``drone_name``."""
        drone_name = self.rpc_client.call('getName')
        if isinstance(drone_name, bytes):
            drone_name = drone_name.decode('utf-8')
        if not isinstance(drone_name, str):
            raise RuntimeError(
                "Incompatible simulator RPC response: getName must return a string, "
                f"got {type(drone_name).__name__}."
            )

        self.drone_name = drone_name
        return self.drone_name

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
        The simulator returns ``range_min`` (or zero) when a ray does not
        intersect an obstacle; the client represents such readings as
        ``range_max``.
 
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
        
        raw_laser_scan_data = self.rpc_client.call(
            'getLaserScan',
            angle_min,
            angle_max,
            range_min,
            range_max,
            num_ranges,
        )
        laser_scan_data = np.asarray(raw_laser_scan_data, dtype=float).reshape(-1)
        laser_scan_data[laser_scan_data <= range_min] = range_max
        
        if not is_clear and len(laser_scan_data) == num_ranges:
            noise = np.random.normal(0, range_error, num_ranges)
            laser_scan_data += noise

        
        return laser_scan_data

    def get_lidar_point_cloud(
        self,
        angle_below_zero: float = np.deg2rad(15.0),
        angle_above_zero: float = np.deg2rad(15.0),
        range_min: float = 0.1,
        range_max: float = 100.0,
        channel_count: int = 16,
        points_per_channel: int = 512,
    ) -> PointCloud:
        """Acquire one complete organized 3D lidar scan.

        Angles are positive magnitudes in radians below and above the horizontal
        plane. The scan always covers 360 degrees horizontally. One method call
        starts exactly one simulator scan and waits until all trace batches have
        completed.

        Returned coordinates are sensor-local metres in ROS FLU convention:
        +X forward, +Y left, +Z up.
        """
        numeric_values = (
            angle_below_zero,
            angle_above_zero,
            range_min,
            range_max,
        )
        if not all(np.isfinite(float(value)) for value in numeric_values):
            raise ValueError("3D lidar angles and ranges must be finite")
        if not 0.0 <= angle_below_zero < np.pi / 2:
            raise ValueError("angle_below_zero must be in [0, pi/2)")
        if not 0.0 <= angle_above_zero < np.pi / 2:
            raise ValueError("angle_above_zero must be in [0, pi/2)")
        if range_min < 0.0 or range_max <= range_min:
            raise ValueError("range_max must be greater than non-negative range_min")
        if isinstance(channel_count, bool) or not isinstance(channel_count, (int, np.integer)):
            raise TypeError("channel_count must be an integer")
        if isinstance(points_per_channel, bool) or not isinstance(points_per_channel, (int, np.integer)):
            raise TypeError("points_per_channel must be an integer")
        if channel_count <= 0 or channel_count > 65_536:
            raise ValueError("channel_count must be in [1, 65536]")
        if points_per_channel <= 0:
            raise ValueError("points_per_channel must be positive")
        if channel_count * points_per_channel > 1_048_576:
            raise ValueError("one 3D lidar scan cannot exceed 1048576 rays")

        response = self.rpc_client.call(
            "getLidarPointCloud",
            float(angle_below_zero),
            float(angle_above_zero),
            float(range_min),
            float(range_max),
            int(channel_count),
            int(points_per_channel),
        )
        return _decode_point_cloud(response)
    
    def get_radar_point(self,
                        radar_id : int = 0,
                        base_angle : float = 45,
                        range_min : float = 0.15,
                        range_max: float = 5,
                        is_clear : bool = True,
                        range_error: float = 0.15,
                        angle_error: float = 0.015):
        
        """
        This function returns the distance to the nearest point that is within
        the radar coverage cone.


        Args:
            radar_id (int) : id of radar
            base_angle (float): cone apex angle
            range_min  (float) : min range for scan distance(meters)
            range_max (float) : max range for scan distance(meters)
            is_clear (bool) : default True, if False is selected, noise will be generated
            range_error (float) : maximum error variation of distance(if is_clear is false)
            angle_error (float) : retained for backwards compatibility; the
                                simulator returns no angle data

        Returns:
            float : point distance(meters). A negative value means that no
                    point was detected.
        """
        
        raw_radar_point = self.rpc_client.call('getRadarData',
                                               radar_id,
                                               base_angle,
                                               range_min,
                                               range_max)

        if not np.isscalar(raw_radar_point):
            raise RuntimeError(
                "Incompatible simulator RPC response: getRadarData must return "
                f"a single distance value, got {raw_radar_point!r}."
            )

        radar_point = float(raw_radar_point)

        if not is_clear:
            radar_point += float(np.random.normal(0, range_error))

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

    def set_Diod(self,
                 led_id, r, g, b):
        return self.rpc_client.call("setDiod", led_id, r, g, b)

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
