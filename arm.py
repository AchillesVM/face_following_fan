from adafruit_servokit import ServoKit
from time import sleep

FAN_IDX = 11


def arm():
    """ Executes the arming procedure for brushless motors.

    For safety, some brushless must be armed before use. This method arms the motor by setting 0% power for two seconds,
    then 100% power for two seconds and finally back to 0%.
    """

    kit = ServoKit(channels=16)

    kit.servo[FAN_IDX].angle = 0
    sleep(2)

    kit.servo[FAN_IDX].angle = 180
    sleep(2)

    kit.servo[FAN_IDX].angle = 0

    print("Motor armed!")


if __name__ == '__main__':

    arm()
