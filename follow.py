from cv2 import CascadeClassifier, CASCADE_FIND_BIGGEST_OBJECT
from adafruit_servokit import ServoKit
from numpy import frombuffer, uint8
from picamera import PiCamera
from time import sleep

CASCADE_PATH = 'haarcascade_frontalface_default.xml'

PROC_RES = (256, 192)  # image processing resolution
FOV = (62.2, 48.8)  # field-of-view of the camera

DEADZONE_RANGE = (0.1, 0.075)  # horizontal deadzone percentage
DEADZONE_OFFSET = (0, 0.05)  # horizontal deadzone offset

FAN_IDX = 11  # pin index of the fan motor
FAN_SPEED = 50  # speed of the fan (0 -> 180)

SERVO_PWM_RANGE = (400, 2600)  # pwm range of the servos

PAN_IDX = 3  # pin index of the pan servo
PAN_CENTRE = 90  # starting position of the pan servo
PAN_RANGE = (40, 140)  # operating range of the pan servo

TILT_IDX = 7  # pin index of the tilt servo
TILT_CENTRE = 150  # starting position of the tilt servo
TILT_RANGE = (100, 180)  # operating range of the tile servo


class Follower(object):
    """ Class for basic object (face) detection and tracking, intended for use with a PiCamera v2 and PCA9685"""

    def __init__(self):

        # get the number of degrees per pixel
        self.dpp = (FOV[0] / PROC_RES[0], FOV[1] / PROC_RES[1])

        # get dead-zone limits in pixels relative to centre
        self.y_dz_lim = (- PROC_RES[1] * (DEADZONE_RANGE[1] - DEADZONE_OFFSET[1]),
                         PROC_RES[1] * (DEADZONE_RANGE[1] + DEADZONE_OFFSET[1]))
        self.x_dz_lim = (- PROC_RES[0] * (DEADZONE_RANGE[0] - DEADZONE_OFFSET[0]),
                         PROC_RES[0] * (DEADZONE_RANGE[0] + DEADZONE_OFFSET[0]))

        # load classification model
        self.cascade = CascadeClassifier(CASCADE_PATH)

        # centre servos and start fan
        self.kit = self.init_servos()
        self.kit.servo[FAN_IDX].angle = 50

    def write(self, buf):
        """ This method is called immediately after each image capture, performs the object detection and adjusts the
        servo positions to follow the object.

        The greyscale image is read directly from the buffer and loaded into a numpy array. The Haar cascade identifies
        the largest object in the image and adjust the servo positions accordingly.

        :param buf: PiCamera memory buffer
        """

        # load greyscale image
        img = frombuffer(buf, dtype=uint8, count=PROC_RES[0] * PROC_RES[1]).reshape((PROC_RES[1], PROC_RES[0]))

        # get largest object
        objs = self.cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=5, flags=CASCADE_FIND_BIGGEST_OBJECT)

        if len(objs) > 0:

            centre = self.get_centre(objs[0])

            # get displacement from centre of image in pixels
            y_disp_px = centre[1] - PROC_RES[1] / 2
            x_disp_px = centre[0] - PROC_RES[0] / 2

            # calculate displacement in degrees and apply fudge factor
            y_disp_deg = y_disp_px * self.dpp[1] * 0.25 * -1  # flip pan axis only
            x_disp_deg = x_disp_px * self.dpp[0] * 0.35

            # if centre of object is outside deadzone, move servos to adjust
            if not self.y_dz_lim[0] < y_disp_px < self.y_dz_lim[1]:
                self.kit.servo[TILT_IDX].angle = max(TILT_RANGE[0], min(TILT_RANGE[1], int(
                    self.kit.servo[TILT_IDX].angle - y_disp_deg)))

            if not self.x_dz_lim[0] < x_disp_px < self.x_dz_lim[1]:
                self.kit.servo[PAN_IDX].angle = max(PAN_RANGE[0],
                                                    min(PAN_RANGE[1], int(self.kit.servo[PAN_IDX].angle - x_disp_deg)))

    def flush(self):
        """ Executes when the recording is terminated, setting the fan speed to 0 for safety.
        """

        self.kit.servo[FAN_IDX] = 0

    @staticmethod
    def init_servos():
        """ Connects to the PCA9685 module via i2c, configures the servo PWM range and centres the pan and tilt servos.

        :return: The adafruit ServoKit object
        """

        # init i2c connection to PCA9685
        kit = ServoKit(channels=16)

        # set fan speed to 0
        kit.servo[FAN_IDX].angle = 0

        # set min and max duty cycle
        kit.servo[PAN_IDX].set_pulse_width_range(SERVO_PWM_RANGE[0], SERVO_PWM_RANGE[1])
        kit.servo[TILT_IDX].set_pulse_width_range(SERVO_PWM_RANGE[0], SERVO_PWM_RANGE[1])

        # centre servos
        kit.servo[PAN_IDX].angle = PAN_CENTRE
        kit.servo[TILT_IDX].angle = TILT_CENTRE

        return kit

    @staticmethod
    def get_centre(obj):
        """ Given the position and size of a bounding box, return the pixel coordinates of the centre of that box.

        :param obj: x and y coords of the bottom left corner, width and height of the bounding box
        :return: x and y coords of the centre of hte bounding box
        """

        x, y, w, h = obj

        return x + (w / 2), y + (h / 2)


def follow():

    with PiCamera(sensor_mode=4, resolution=f"{PROC_RES[0]}x{PROC_RES[1]}", framerate=40) as camera:

        # wait for camera to init
        sleep(2)

        # instantiate follower class
        follower = Follower()

        # follow for 1000 seconds
        camera.start_recording(follower, 'yuv')
        camera.wait_recording(1000)
        camera.stop_recording()


if __name__ == "__main__":
    follow()
