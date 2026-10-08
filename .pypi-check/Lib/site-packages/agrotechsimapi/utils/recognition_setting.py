import cv2.aruco as aruco

import numpy as np

fx = 150  # x-axis focal length
fy = 150 # н-axis focal length
cx = 320.0 / 2  # optical center x-axis
cy = 240.0 / 2  # optical center н-axis

camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

k1 = 0.1  # radial distortion factor
k2 = 0.01  # second radial distortion factor
p1 = 0.001  # first tangential distortion factor
p2 = 0.002  # second tangential distortion factor

distance_coefficients = np.array([k1, k2, p1, p2, 0], dtype=np.float32)

marker_size = 0.5

#aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # для новой комнаты
#aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # для 1 комнаты из обычного симулятора 

aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

detector_parameters = aruco.DetectorParameters() 