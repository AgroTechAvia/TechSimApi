import cv2.aruco as aruco

import numpy as np

fx = 1000.0  # x-axis focal length
fy = 1000.0  # н-axis focal length
cx = 480.0   # optical center x-axis
cy = 360.0   # optical center н-axis

camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

k1 = 0.1  # radial distortion factor
k2 = 0.01  # second radial distortion factor
p1 = 0.001  # first tangential distortion factor
p2 = 0.002  # second tangential distortion factor

distance_coefficients = np.array([k1, k2, p1, p2, 0], dtype=np.float32)

marker_size = 0.05


aruco_dictionary = aruco.getPredefinedDictionary(dict = aruco.DICT_ARUCO_ORIGINAL)

detector_parameters = aruco.DetectorParameters() 