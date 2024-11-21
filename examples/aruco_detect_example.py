from agrotechsimapi import SimClient
import time
import cv2

from aruco_marker_recognizer import ArucoRecognizer
from recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

#python -m examples.aruco_detect_example
def main():

    aruco_recognizer = ArucoRecognizer(aruco_dictionary = aruco_dictionary,
                                                marker_size = marker_size,
                                                distance_coefficients = distance_coefficients,
                                                detector_parameters = detector_parameters,
                                                camera_matrix = camera_matrix)
    
    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    while is_loop:  
        result = client.get_camera_capture(camera_id = 0, is_clear=True)
        
        cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = aruco_recognizer.detect_aruco_markers(result)

        if cv_image_with_markers is not None:
            if (cv_image_with_markers.shape[0] > 0) and (cv_image_with_markers.shape[1] > 0):
                result = cv_image_with_markers
                
        cv2.imshow(f"Capture from  camera", result)
        

        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        time.sleep(1/20)


main()