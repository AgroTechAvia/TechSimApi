import cv2
import numpy as np
import math
from collections import deque
from typing import Dict, List, Optional

from .aruco_marker_recognizer import ArucoRecognizer
from .recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

aruco_recognizer = ArucoRecognizer(
    aruco_dictionary=aruco_dictionary,
    marker_size=marker_size,
    distance_coefficients=distance_coefficients,
    detector_parameters=detector_parameters,
    camera_matrix=camera_matrix
)


def _clone_pose(pose: dict) -> dict:
    return {
        "position": {
            "x": float(pose["position"]["x"]),
            "y": float(pose["position"]["y"]),
            "z": float(pose["position"]["z"]),
        },
        "orientation": {
            "x": float(pose["orientation"]["x"]),
            "y": float(pose["orientation"]["y"]),
            "z": float(pose["orientation"]["z"]),
        },
    }


def _median_pose(pose_history: deque) -> Optional[dict]:
    if not pose_history:
        return None

    pose = {"position": {}, "orientation": {}}
    for section in ("position", "orientation"):
        for axis in ("x", "y", "z"):
            values = [entry[section][axis] for entry in pose_history]
            pose[section][axis] = float(np.median(values))
    return pose


def _ema_pose(previous_pose: Optional[dict], current_pose: Optional[dict], alpha: float) -> Optional[dict]:
    if current_pose is None:
        return None
    if previous_pose is None:
        return _clone_pose(current_pose)

    filtered_pose = {"position": {}, "orientation": {}}
    for section in ("position", "orientation"):
        for axis in ("x", "y", "z"):
            previous_value = previous_pose[section][axis]
            current_value = current_pose[section][axis]
            filtered_pose[section][axis] = float((alpha * current_value) + ((1.0 - alpha) * previous_value))
    return filtered_pose


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _extract_relative_yaw(rotation_matrix: np.ndarray) -> float:
    """
    Возвращает yaw-like угол из относительной ориентации.

    Нельзя брать rvec[2] напрямую: это компонент вектора Родрига, а не yaw.
    Вместо этого берем нормаль к плоскости маркера и смотрим ее проекцию
    на горизонтальную плоскость XZ камеры.
    """
    normal_x = -float(rotation_matrix[0, 2])
    normal_z = -float(rotation_matrix[2, 2])

    if abs(normal_x) < 1e-9 and abs(normal_z) < 1e-9:
        return 0.0

    return _wrap_angle(math.atan2(normal_x, normal_z))


def _extract_marker_orientation(rotation_matrix: np.ndarray) -> dict:
    """
    Возвращает осмысленные углы ориентации маркера относительно камеры.

    Это НЕ Rodrigues vector и не классические Euler углы объекта в мировой СК.
    Для задач визуального наведения нам удобнее:
    - x: roll-like угол поворота маркера в плоскости изображения
    - y: pitch-like угол наклона маркера вверх/вниз
    - z: yaw-like угол разворота маркера влево/вправо
    """
    normal = -rotation_matrix[:, 2]
    up_axis = rotation_matrix[:, 1]

    normal_x = float(normal[0])
    normal_y = float(normal[1])
    normal_z = float(normal[2])

    yaw_like = _wrap_angle(math.atan2(normal_x, normal_z))
    pitch_like = _wrap_angle(math.atan2(-normal_y, math.hypot(normal_x, normal_z)))
    roll_like = _wrap_angle(math.atan2(float(up_axis[0]), -float(up_axis[1])))

    return {
        "x": float(roll_like),
        "y": float(pitch_like),
        "z": float(-yaw_like),
    }


class _ArucoMarkerTrack:
    def __init__(self, marker_id: int, visibility_window: int, pose_window: int):
        self.marker_id = int(marker_id)
        self.visibility_history = deque(maxlen=visibility_window)
        self.marker_pose_history = deque(maxlen=pose_window)
        self.camera_pose_history = deque(maxlen=pose_window)
        self.filtered_marker_pose: Optional[dict] = None
        self.filtered_camera_pose: Optional[dict] = None
        self.consecutive_seen = 0
        self.missed_frames = 0
        self.visible_now = False
        self.is_stable = False

    def observe(self, marker: Optional[dict], camera_pose: Optional[dict], ema_alpha: float):
        self.visible_now = marker is not None
        self.visibility_history.append(self.visible_now)

        if self.visible_now:
            self.consecutive_seen += 1
            self.missed_frames = 0

            self.marker_pose_history.append(_clone_pose(marker["pose"]))
            marker_median_pose = _median_pose(self.marker_pose_history)
            self.filtered_marker_pose = _ema_pose(self.filtered_marker_pose, marker_median_pose, ema_alpha)

            if camera_pose is not None:
                self.camera_pose_history.append(_clone_pose(camera_pose["pose"]))
                camera_median_pose = _median_pose(self.camera_pose_history)
                self.filtered_camera_pose = _ema_pose(self.filtered_camera_pose, camera_median_pose, ema_alpha)
        else:
            self.consecutive_seen = 0
            self.missed_frames += 1

    def visible_ratio(self) -> float:
        if not self.visibility_history:
            return 0.0
        return sum(1 for value in self.visibility_history if value) / len(self.visibility_history)

    def should_prune(self) -> bool:
        if not self.visibility_history:
            return False
        return all(not value for value in self.visibility_history)


class StableArucoTracker:
    def __init__(
        self,
        visibility_window: int = 10,
        min_visible_ratio: float = 0.7,
        min_consecutive_frames: int = 3,
        min_samples_for_stable: int = 5,
        stable_exit_ratio: float = 0.4,
        max_missing_frames: int = 2,
        pose_window: int = 5,
        pose_ema_alpha: float = 0.35,
    ):
        self._visibility_window = max(3, int(visibility_window))
        self._min_visible_ratio = float(min_visible_ratio)
        self._min_consecutive_frames = max(1, int(min_consecutive_frames))
        self._min_samples_for_stable = max(1, int(min_samples_for_stable))
        self._stable_exit_ratio = float(stable_exit_ratio)
        self._max_missing_frames = max(1, int(max_missing_frames))
        self._pose_window = max(1, int(pose_window))
        self._pose_ema_alpha = float(pose_ema_alpha)
        self._tracks: Dict[int, _ArucoMarkerTrack] = {}

    def reset(self):
        self._tracks.clear()

    def update(self, aruco_data: List[dict], camera_pose_aruco: List[dict]) -> tuple[list[dict], list[dict]]:
        raw_markers_by_id = {int(marker["id"]): marker for marker in aruco_data}
        raw_camera_pose_by_id = {int(marker["id"]): marker for marker in camera_pose_aruco}

        stable_entries = []
        known_ids = set(self._tracks.keys()) | set(raw_markers_by_id.keys()) | set(raw_camera_pose_by_id.keys())

        for marker_id in known_ids:
            track = self._tracks.get(marker_id)
            if track is None:
                track = _ArucoMarkerTrack(
                    marker_id=marker_id,
                    visibility_window=self._visibility_window,
                    pose_window=self._pose_window,
                )
                self._tracks[marker_id] = track

            raw_marker = raw_markers_by_id.get(marker_id)
            raw_camera_pose = raw_camera_pose_by_id.get(marker_id)
            track.observe(raw_marker, raw_camera_pose, self._pose_ema_alpha)

            self._update_stability(track)
            if track.should_prune():
                del self._tracks[marker_id]
                continue

            if track.is_stable and track.visible_now and track.filtered_marker_pose is not None:
                score = track.visible_ratio()
                stable_entries.append({
                    "score": score,
                    "marker": self._build_marker_output(marker_id, track.filtered_marker_pose, track, score),
                    "camera_pose": self._build_marker_output(marker_id, track.filtered_camera_pose, track, score)
                    if track.filtered_camera_pose is not None else None,
                })

        stable_entries.sort(
            key=lambda entry: (
                entry["score"],
                entry["marker"]["tracking"]["consecutive_seen"],
                -entry["marker"]["id"],
            ),
            reverse=True,
        )

        stable_markers = [entry["marker"] for entry in stable_entries]
        stable_camera_pose = [entry["camera_pose"] for entry in stable_entries if entry["camera_pose"] is not None]
        return stable_markers, stable_camera_pose

    def _update_stability(self, track: _ArucoMarkerTrack):
        visible_ratio = track.visible_ratio()
        enough_history = len(track.visibility_history) >= self._min_samples_for_stable

        if not track.is_stable:
            if (
                track.visible_now
                and enough_history
                and track.consecutive_seen >= self._min_consecutive_frames
                and visible_ratio >= self._min_visible_ratio
            ):
                track.is_stable = True
        else:
            if (
                track.missed_frames > self._max_missing_frames
                or (enough_history and visible_ratio < self._stable_exit_ratio)
            ):
                track.is_stable = False

    def _build_marker_output(self, marker_id: int, pose: dict, track: _ArucoMarkerTrack, score: float) -> dict:
        return {
            "id": int(marker_id),
            "pose": _clone_pose(pose),
            "tracking": {
                "stable": True,
                "stable_score": round(float(score), 3),
                "visible_ratio": round(float(track.visible_ratio()), 3),
                "consecutive_seen": int(track.consecutive_seen),
                "missed_frames": int(track.missed_frames),
                "window_fill": int(len(track.visibility_history)),
                "visible_now": bool(track.visible_now),
            },
        }



def process_blob(img):
    min_area=100    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # HSV для тренировок
    # color_ranges = {
    # '0': [   # red
    #     (np.array([0, 80, 100]), np.array([0, 255, 255]))
    # ],
    # '1': [   # green
    #     (np.array([30, 60, 200]), np.array([45, 130, 255])),  
    # ],
    # '2': [   # blue  
    #     (np.array([95, 100, 80]), np.array([115, 255, 200])),  
    # ]
    # }
    
    # HSV для отборочного этапа
    color_ranges = {
    '0': [   # red
        (np.array([0, 80, 90]), np.array([13, 255, 255]))
    ],
    '1': [   # green
        (np.array([30, 150, 150]), np.array([70, 255, 255])),  
    ],
    '2': [   # blue  
        (np.array([90, 100, 100]), np.array([170, 255, 255])),  
    ]
    }

    blobs = []  
    all_contours_info = []
    blob_img = None

    for color_name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            color_mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_or(mask, color_mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else:
                center_x = x + w // 2
                center_y = y + h // 2
            
            blob_data = {
                'id': color_name,  
                'center': {'x': center_x, 'y': center_y},
                'size': {'x': w, 'y': h},
            }
            blobs.append(blob_data)
            
            all_contours_info.append((contour, color_name, center_x, center_y))


            blob_img = draw_blobs_on_image(img, all_contours_info)

    
    return blobs, blob_img 



def draw_blobs_on_image(img, contours_info):
    color_map = {
        '0': (0, 0, 255), #red
        '1': (0, 255, 0), #green
        '2': (255, 0, 0) #blue
    }
    for contour, color_name, center_x, center_y in contours_info:
        color_bgr = color_map[color_name]
        
        cv2.drawContours(img, [contour], -1, color_bgr, 2)
        cv2.circle(img, (center_x, center_y), 4, color_bgr, -1)
        
    return img
    
    

def process_aruco(img):

    aruco_data = []
    camera_pose_aruco = []
    cv_image_with_markers = img
    
    if img is not None and len(img) != 0:
        cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(img)
        if markers_ids is not None:
            for i in range(len(markers_ids)):
                marker_id = markers_ids[i][0]
                tvec = translation_vectors[i].flatten()  
                rvec = rotation_vectors[i].flatten()
                R, _ = cv2.Rodrigues(rvec)
                marker_orientation = _extract_marker_orientation(R)

                # Маркер относительно камеры
                position_data = tvec
                aruco_data.append({
                    'id': int(marker_id),
                    'pose': {
                        'position': {
                            'x': float(position_data[0]),
                            'y': float(position_data[1]), 
                            'z': float(position_data[2])
                        },
                        'orientation': marker_orientation
                    }
                })


                # Камера относительно маркера
                camera_position = -np.dot(R.T, tvec)
                camera_rotation = R.T
                camera_orientation = _extract_marker_orientation(camera_rotation)
                
                position_data = camera_position
                
                camera_pose_aruco.append({
                    'id': int(marker_id),
                    'pose': {
                        'position': {
                            'x': float(position_data[0]),
                            'y': float(position_data[1]), 
                            'z': float(position_data[2])
                        },
                        'orientation': camera_orientation
                    }
                })

    return aruco_data, camera_pose_aruco, cv_image_with_markers

def resolution_changes(img, new_size):
     image = cv2.resize(img, new_size, interpolation=cv2.INTER_LINEAR)
     return image
