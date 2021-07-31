import time
import cv2 as cv
from adafruit_servokit import ServoKit
import numpy as np
import picamera

PROC_RES = (256, 192)
H_FOV = 62.2
V_FOV = 48.8
PAN_IDX = 3
TILT_IDX = 7
FAN_IDX = 11
PAN_RANGE = (40, 140)
TILT_RANGE = (100, 180)


def get_centre(face):

    x, y, w, h = face

    return x + (w / 2), y + (h / 2)


def init_servos():

    kit = ServoKit(channels=16)

    kit.servo[FAN_IDX].angle = 0

    kit.servo[PAN_IDX].set_pulse_width_range(400, 2600)
    kit.servo[TILT_IDX].set_pulse_width_range(400, 2600)

    kit.servo[PAN_IDX].angle = 90
    kit.servo[TILT_IDX].angle = 160

    return kit


class Output(object):
    
    def __init__(self, face_cascade, kit):
        self.face_cascade = face_cascade
        self.kit = kit  
        self.t0 = time.time()
        self.count = 0
        self.y_dpp = V_FOV / PROC_RES[1]  
        self.x_dpp = H_FOV / PROC_RES[0]

    def write(self, buf):

        img = np.frombuffer(buf, dtype=np.uint8, count=PROC_RES[0]*PROC_RES[1]).reshape((PROC_RES[1], PROC_RES[0]))

        faces = self.face_cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=5, flags=cv.CASCADE_FIND_BIGGEST_OBJECT)

        if len(faces) > 0:
            centre = get_centre(faces[0])
            y_diff = centre[1] - PROC_RES[1] / 2
            x_diff = centre[0] - PROC_RES[0] / 2
            y_factor = y_diff * 0.25
            x_factor = x_diff * 0.35

            if y_diff < -5 or y_diff > 25:
                self.kit.servo[TILT_IDX].angle = max(TILT_RANGE[0], min(TILT_RANGE[1], int(self.kit.servo[TILT_IDX].angle + y_factor*self.y_dpp)))

            if x_diff > 25 or x_diff < -25:
                self.kit.servo[PAN_IDX].angle = max(PAN_RANGE[0], min(PAN_RANGE[1], int(self.kit.servo[PAN_IDX].angle - x_factor*self.x_dpp)))

    def flush(self):
        pass


def follow():

    kit = init_servos()

    kit.servo[FAN_IDX].angle = 50

    with picamera.PiCamera(sensor_mode=4, resolution=f"{PROC_RES[0]}x{PROC_RES[1]}", framerate=40) as camera:
        time.sleep(2)
        face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
        output = Output(face_cascade, kit)
        camera.start_recording(output, 'yuv')
        camera.wait_recording(1000)
        camera.stop_recording()


if __name__ == "__main__":

    follow()
