import cv2
import numpy as np

from .aruco_marker_recognizer import ArucoRecognizer
from .recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

aruco_recognizer = ArucoRecognizer(
    aruco_dictionary=aruco_dictionary,
    marker_size=marker_size,
    distance_coefficients=distance_coefficients,
    detector_parameters=detector_parameters,
    camera_matrix=camera_matrix
)



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
    
    if img is not None and len(img) != 0:
        cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(img)
        if markers_ids is not None:
            for i in range(len(markers_ids)):
                marker_id = markers_ids[i][0]
                tvec = translation_vectors[i].flatten()  
                rvec = rotation_vectors[i].flatten()

                # Маркер относительно камеры
                position_data = tvec
                orientation_data = rvec
                aruco_data.append({
                    'id': int(marker_id),
                    'pose': {
                        'position': {
                            'x': float(position_data[0]),
                            'y': float(position_data[1]), 
                            'z': float(position_data[2])
                        },
                        'orientation': {
                            'x': float(orientation_data[0]),
                            'y': float(orientation_data[1]), 
                            'z': -1 * float(orientation_data[2])
                        }
                    }
                })


                # Камера относительно маркера
                R, _ = cv2.Rodrigues(rvec)
                camera_position = -np.dot(R.T, tvec)
                camera_rvec, _ = cv2.Rodrigues(R.T)
                
                position_data = camera_position
                orientation_data = camera_rvec.flatten()
                
                camera_pose_aruco.append({
                    'id': int(marker_id),
                    'pose': {
                        'position': {
                            'x': float(position_data[0]),
                            'y': float(position_data[1]), 
                            'z': float(position_data[2])
                        },
                        'orientation': {
                            'x': float(orientation_data[0]),
                            'y': float(orientation_data[1]), 
                            'z': float(orientation_data[2])
                        }
                    }
                })

    return aruco_data, camera_pose_aruco, cv_image_with_markers

def resolution_changes(img, new_size):
     image = cv2.resize(img, new_size, interpolation=cv2.INTER_LINEAR)
     return image