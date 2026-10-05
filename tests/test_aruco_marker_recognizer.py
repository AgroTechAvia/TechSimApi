import cv2
import cv2.aruco as aruco
import numpy as np
import pytest

from agrotechsimapi.utils.aruco_marker_recognizer import ArucoRecognizer


@pytest.mark.parametrize(
    ("dictionary_id", "marker_id"),
    [
        (aruco.DICT_4X4_50, 17),
        (aruco.DICT_ARUCO_ORIGINAL, 42),
    ],
)
def test_detect_aruco_markers_returns_id_pose_and_annotated_frame(dictionary_id, marker_id):
    """The detector must recognise supported TechSim marker dictionaries and annotate the frame."""
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    marker = aruco.generateImageMarker(dictionary, marker_id, 200)
    image = np.full((300, 300), 255, dtype=np.uint8)
    image[50:250, 50:250] = marker
    frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    original_frame = frame.copy()

    recognizer = ArucoRecognizer(
        aruco_dictionary=dictionary,
        marker_size=0.1,
        distance_coefficients=np.zeros(5, dtype=np.float32),
        detector_parameters=aruco.DetectorParameters(),
        camera_matrix=np.array(
            [[300.0, 0.0, 150.0], [0.0, 300.0, 150.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        ),
    )

    annotated_frame, marker_ids, rotation_vectors, translation_vectors = recognizer.detect_aruco_markers(frame)

    assert marker_ids.flatten().tolist() == [marker_id]
    assert len(rotation_vectors) == 1
    assert len(translation_vectors) == 1
    assert annotated_frame.shape == original_frame.shape
    assert not np.array_equal(annotated_frame, original_frame)
