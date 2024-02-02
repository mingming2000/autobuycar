# Refer to `test/shopping_cart/setting/camera.py`
import cv2
import numpy as np

class Camera:
    def __init__(
        self, 
        device_idx: int = 0,
        lower: np.ndarray = np.array([ 0, 150, 140]),
        upper: np.ndarray = np.array([50, 255, 255]),
    ):
        self.lower = lower
        self.upper = upper
        self.device_idx = 0

    def initializing(self):

        while True:
            webcam_video = cv2.VideoCapture(self.device_idx)
            success, video = webcam_video.read()
            img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
            img_h, img_w, img_c = img.shape  

            mask = cv2.inRange(img, self.lower, self.upper) 
            mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
            
            if len(mask_contours) != 0:
                for mask_contour in mask_contours:
                    if cv2.contourArea(mask_contour) > 1000:
                        x, y, w, h = cv2.boundingRect(mask_contour)
                        if (x + w/2 > img_w * 0.4 and x + w/2 < img_w * 0.6):
                            print("camera Ready done")
                            webcam_video.release()
                            return True
            

    def calculate_angle(self) -> float | None:
        webcam_video2 = cv2.VideoCapture(self.device_idx)
        _, video = webcam_video2.read()
        img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
        img_h, img_w, img_c = img.shape  

        mask = cv2.inRange(img, self.lower, self.upper) 
        mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

        if len(mask_contours) != 0:
            for mask_contour in mask_contours:
                if cv2.contourArea(mask_contour) > 1000:
                    x, y, w, h = cv2.boundingRect(mask_contour)
                    degree = (x + w/2) * 100 / img_w
            return degree
        else:
            degree = 50
            return degree