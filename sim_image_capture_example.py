import agrotechsimapi
import cv2
import cv2.aruco as aruco
import numpy as np

print("[INFO] Connect to simulator...")

IP = "127.0.0.1"
PORT = 41451
isConnected = False
connection_count = 10

client = agrotechsimapi.MultirotorClient(ip = IP, port = PORT)

try:
    client.confirmConnection()
    print("[INFO] Connect successful")
    isConnected = True
except:
    print("[INFO] Connect fail")


fx = 1000.0  # focal length x
fy = 1000.0  # focal length y
cx = 640.0   # optical center x
cy = 480.0   # optical center y

# Создание матрицы камеры
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

k1 = 0.1  # first radial distortion coefficient
k2 = 0.01  # second radial distortion coefficient
p1 = 0.001  # first tan distortion coefficient 
p2 = 0.002  # second tan distortion coefficient

dist_coeffs = np.array([k1, k2, p1, p2, 0], dtype=np.float32)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

parameters = aruco.DetectorParameters() 


def detect_aruco_markers(frame,dict, parameters):
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image=frame, dictionary=dict, parameters=parameters)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        for i in range(len(ids)):

            frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.025)

            c = corners[i][0]
            x = int(np.mean(c[:, 0]))
            y = int(np.mean(c[:, 1]))
            cv2.putText(frame,'id:' + str(ids[i]), (x+30, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (220,180,0), 2, cv2.LINE_AA)

    return frame

def resize_img(percent,img):   
	width = int(img.shape[1] * percent / 100)
	height = int(img.shape[0] * percent / 100)
	dim = (width, height)

	return cv2.resize(img, dim, interpolation=cv2.INTER_LANCZOS4)

while isConnected:
    rawImage = client.simGetImage("0", agrotechsimapi.ImageType.Scene)
    if rawImage != None:
        frame = cv2.imdecode(agrotechsimapi.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)

        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

        acuro_frame = detect_aruco_markers(frame,aruco_dict, parameters)
        cv2.imshow("image", acuro_frame)
    
    key = cv2.waitKey(1) & 0xFF
    if (key == 27 or key == ord('q') or key == ord('x')):
        break

