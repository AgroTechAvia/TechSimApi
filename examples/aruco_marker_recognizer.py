import cv2
import cv2.aruco as aruco

import numpy as np

from typing import Tuple

class ArucoRecognizer:
    
    def __init__(self, aruco_dictionary: aruco.DetectorParameters, marker_size: float, distance_coefficients: np.ndarray, 
                 detector_parameters: np.ndarray, camera_matrix: np.ndarray) -> None:
        """
        When initializing an instance of the class, 
        parameters for the detector are passed, 
        as well as data that allows the function 
        to determine the distance and position of the markers
        """
        
        self.aruco_dictionary = aruco_dictionary
        self.marker_size = marker_size
        self.distance_coefficients = distance_coefficients
        self.detector_parameters = detector_parameters
        self.camera_matrix = camera_matrix

        self.detector = aruco.ArucoDetector(aruco_dictionary, detector_parameters)

    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def detect_aruco_markers(self, frame: np.ndarray) -> Tuple[np.ndarray,np.ndarray,np.ndarray,np.ndarray]:
        """
        This function recognizes markers on an image, draws local coordinate systems on it, 
        as well as information (ID and distance). 
        Returns information about markers and rotations 
        and translations matrices if they are recognized, otherwise none

        Args:
            frame(np.array): openCV image from airsim

        Returns:
            ndarray: openCV image   
            ndarray: recognized marker ids      
            ndarray: rotation vector   
            ndarray: translation vector         
        """
        
        markers_corners, markers_ids, rejected_img_points = self.detector.detectMarkers(image = frame) 
                                                                                #dictionary = self.aruco_dictionary, 
                                                                                #parameters = self.detector_parameters) #aruco.detectMarkers
        
        if markers_ids is not None:
            '''rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(corners = markers_corners, 
                                                                                       markerLength = self.marker_size, 
                                                                                       cameraMatrix = self.camera_matrix, 
                                                                                       distCoeffs  = self.distance_coefficients)'''
            
            rotation_vectors, translation_vectors, _ = self.estimatePoseSingleMarkers(corners = markers_corners, 
                                                                                       marker_size = self.marker_size, 
                                                                                       mtx = self.camera_matrix, 
                                                                                       distortion  = self.distance_coefficients)
            for i in range(len(markers_ids)):
                frame_wit_axes = cv2.drawFrameAxes(image = frame, cameraMatrix = self.camera_matrix, 
                                                   distCoeffs = self.distance_coefficients, 
                                                   rvec = rotation_vectors[i], 
                                                   tvec = translation_vectors[i], 
                                                   length= 0.02, 
                                                   thickness=2)
                
                #biases for drowing text, not for calculating 
                x_bias = -25
                y_bias = -15

                marker_corners = markers_corners[i][0]
                x = int(np.mean(marker_corners[:, 0]))
                y = int(np.mean(marker_corners[:, 1]))

                id_text = 'id:' + str(markers_ids[i])
                cv2.putText(img = frame_wit_axes,
                            text = id_text, 
                            org = (x + x_bias, y + y_bias), 
                            fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale = 0.5, 
                            color = (160, 245, 18), #bgr
                            thickness = 1, 
                            lineType = cv2.LINE_AA)
                
                distance_to_marker = translation_vectors[i][2][0]
                dist_text =  'distance: ' + str(round(distance_to_marker,3))

                cv2.putText(img = frame_wit_axes,
                            text = dist_text, 
                            org = (x + x_bias, y - y_bias), 
                            fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                            fontScale = 0.4, 
                            color = (160, 245, 18), 
                            thickness = 1, 
                            lineType = cv2.LINE_AA)
    
            return frame_wit_axes, markers_ids, rotation_vectors, translation_vectors 
        
        else:
        
            return None, None, None, None
        