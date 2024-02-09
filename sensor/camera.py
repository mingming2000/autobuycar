# Refer to `test/shopping_cart/setting/camera.py`
import cv2
import numpy as np
import time
from contextlib import contextmanager


@contextmanager
def measure_runtime():
    stime = time.perf_counter()
    yield
    etime = time.perf_counter()
    print(f">>> Runtime >>>{etime - stime:.2f} (sec)")


class Camera:
    def __init__(
        self, 
        # device_idx: int = 0,
        lower: np.ndarray = np.array([ 6, 120, 48]),
        upper: np.ndarray = np.array([20, 255, 255]),
    ):
        self.device_idx = 0
        self.lower = lower
        self.upper = upper
        self.webcam_video = cv2.VideoCapture(self.device_idx)

    def initializing(self):
        while True:
            # webcam_video = cv2.VideoCapture(self.device_idx)
            success, video = self.webcam_video.read()
            img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
            img_h, img_w, img_c = img.shape  

            mask = cv2.inRange(img, self.lower, self.upper) 
            mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
            
            if len(mask_contours) != 0:
                for mask_contour in mask_contours:
                    if cv2.contourArea(mask_contour) > 500:
                        x, y, w, h = cv2.boundingRect(mask_contour)
                        if (x + w/2 > img_w * 0.4 and x + w/2 < img_w * 0.6):
                            print("camera Ready done")
                            # self.webcam_video.release()
                            return True
                    
    def calculate_angle(self):
        webcam_video2 = cv2.VideoCapture(self.device_idx)
        _, video = webcam_video2.read()
        img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
        img_h, img_w, img_c = img.shape  

        mask = cv2.inRange(img, self.lower, self.upper) 
        mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        degree = 50
        
        if len(mask_contours) != 0:
            for mask_contour in mask_contours:
                if cv2.contourArea(mask_contour) > 1000:
                    x, y, w, h = cv2.boundingRect(mask_contour)
                    degree = (x + w/2) * 100 / img_w
            return degree
        else:
            return degree

    def calculate_servo_degree(self):
        len_max_contour = 0
        max_contour_sel = None
        measured_angle = 50
        mode = "Bluetooth"
        area = None
        sucess, video = self.webcam_video.read()
        if sucess == True:
            img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
            _, img_w, _ = img.shape  

            mask = cv2.inRange(img, self.lower, self.upper) 
            mask_contours, _ = cv2.findContours(
                mask, 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            ) 

            if len(mask_contours) != 0:
                for mask_contour in mask_contours:
                    area = cv2.contourArea(mask_contour)
                    if(len_max_contour < area):
                        max_contour_sel = mask_contour
                        len_max_contour = area

                # print(f">>> Contour : {len_max_contour}")

                if max_contour_sel is not None:
                    x_list = [ node[0][0] for node in max_contour_sel ]
                    x_mean = sum(x_list) / len(x_list)
                    measured_angle = x_mean * 100 / img_w
                    if(len_max_contour > 70000):
                        mode = 'Camera'
        degree_servo = 1300 + measured_angle * 6 
        # print(f">>> mask_area: {area}, degree: {degree_servo:.2f}, mode: {mode} ")
        # return len_max_contour
        return degree_servo, mode

