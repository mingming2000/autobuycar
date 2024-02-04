from sensor.bluetooth import Bluetooth
from sensor.camera import Camera

import RPi.GPIO as GPIO
import pigpio
import cv2

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
    @staticmethod
    def print_blue(text: str):
        print(bcolors.OKBLUE + text + bcolors.ENDC)

    @staticmethod
    def print_green(text: str):
        print(bcolors.OKGREEN + text + bcolors.ENDC)

    @staticmethod
    def print_cyan(text: str):
        print(bcolors.OKCYAN + text + bcolors.ENDC)
    
    @staticmethod
    def print_warning(text: str):
        print(bcolors.WARNING + text + bcolors.ENDC)



servoPin = 17

RPWM = 26	# forward	Physical 37
LPWM = 21	# reverse	Physical 40
R_EN = 19	#                    35
L_EN = 20	#					 38

STOP = 0
FORWARD = 1
BACKWARD = 2
HIGH = 1
LOW = 0

lamda1 = 0.35
lamda2 = 0.65


if __name__ == "__main__":
    # Initialization Servo Motor in Here!
    servo = pigpio.pi()
    servo.set_mode(servoPin, pigpio.OUTPUT)
    servo.set_PWM_frequency(servoPin, 50)

    # Initialization DC Motor in Here!
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(R_EN, GPIO.OUT)
    GPIO.setup(L_EN, GPIO.OUT)
    GPIO.setup(RPWM, GPIO.OUT)
    GPIO.setup(LPWM, GPIO.OUT)

    pwm_r = GPIO.PWM(RPWM, 100)
    pwm_l = GPIO.PWM(LPWM, 100)

    pwm_r.start(0)
    pwm_l.start(0)

    # Initialization MultiTasks and others in Here!
    camera = Camera()
    bluetooth = Bluetooth()

    bcolors.print_green('Initializing Start!')
    camera.initializing()
    bcolors.print_green('Initializing Camera Finish!')
    bluetooth.initializing()
    bcolors.print_green('Initializing Bluetooth Finish!')

    try:
        while True:
            # Measure angle and distance in sequential!
            
            
            # Measure distance and control DC motor!
            measured_distance = bluetooth.calculate_distance()
            bcolors.print_blue(f'>>> Distance: {measured_distance}')

            # Control DC Motor!
            if(measured_distance > lamda2): # Forward
                bcolors.print_green('Forward!')
                direction = 'Forward'
                pwm_r.ChangeDutyCycle(10)
                pwm_l.ChangeDutyCycle(0)
            elif(measured_distance < lamda1): # Backward
                bcolors.print_green('Backward!')
                direction = 'Backward'
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(10)
            else:
                # bcolors.print_warning('Direction Error')
                direction = None
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(0)


            # Measure angle and control servo motor!
            while(True):
                sucess, video = camera.webcam_video.read()
                print(f">>> sucess: {sucess}")
                if(sucess == True):
                    img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) 
                    img_h, img_w, img_c = img.shape  

                    mask = cv2.inRange(img, camera.lower, camera.upper) 
                    mask_contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

                    measured_angle = 50
                    if len(mask_contours) != 0:
                        for mask_contour in mask_contours:
                            print(f">>> mask_contour: {cv2.contourArea(mask_contour)}")
                            if cv2.contourArea(mask_contour) > 500:
                                x, y, w, h = cv2.boundingRect(mask_contour)
                                measured_angle = (x + w/2) * 100 / img_w
                        break

            bcolors.print_blue(f'>>> Angle: {measured_angle}')
            
            # Control Servo Motor!
            degree_servo = 1300 + measured_angle * 6 
            if(direction == 'Forward' or direction == None):
                if(degree_servo < 1500) :
                    bcolors.print_green('Turn Left')
                    servo.set_servo_pulsewidth(servoPin, degree_servo)
                elif (degree_servo > 1700 ):
                    bcolors.print_green('Turn Right')
                    servo.set_servo_pulsewidth(servoPin, degree_servo)
                else:
                    servo.set_servo_pulsewidth(servoPin, 1600)
            
            elif(direction == 'Backward'):
                if(degree_servo < 1500) :
                    bcolors.print_green('Turn Left')
                    servo.set_servo_pulsewidth(servoPin, 3200 - degree_servo)
                elif (degree_servo > 1700 ):
                    bcolors.print_green('Turn Right')
                    servo.set_servo_pulsewidth(servoPin, 3200 - degree_servo)
                else:
                    servo.set_servo_pulsewidth(servoPin, 1600)

            else:
                servo.set_servo_pulsewidth(servoPin, 1600)

    except KeyboardInterrupt as e: # Terminate GPIO and Servo Pins Safety!
        pwm_r.stop()
        pwm_l.stop()
        GPIO.cleanup()
        camera.webcam_video.release()
        servo.set_PWM_dutycycle(servoPin, 0)
        servo.set_PWM_frequency(servoPin, 0)
        servo.stop()
    finally: # Terminate GPIO and Servo Pins Safety!
        pwm_r.stop()
        pwm_l.stop()
        GPIO.cleanup()
        camera.webcam_video.release()
        servo.set_PWM_dutycycle(servoPin, 0)
        servo.set_PWM_frequency(servoPin, 0)
        servo.stop()