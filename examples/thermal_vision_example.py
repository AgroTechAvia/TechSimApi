from agrotechsimapi import SimClient
import time
import cv2



def main():
    is_loop = True
    client = SimClient(address = "127.0.0.1", port = 8080)

    while is_loop:  
        result = client.get_camera_capture(camera_id = 0, is_clear = True, is_thermal = True)
        
        if  result is not None:
            if len(result) != 0:
                cv2.imshow(f"Capture from  camera", result)

        if cv2.waitKey(1) == ord('q'):
            is_loop = False
            cv2.destroyAllWindows()

        time.sleep(1/30)


main()