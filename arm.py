from adafruit_servokit import ServoKit
from time import sleep

FAN_IDX = 11

if __name__ == '__main__':

    kit = ServoKit(channels=16)

    kit.servo[FAN_IDX].angle = 0
    sleep(2)

    kit.servo[FAN_IDX].angle = 180
    sleep(2)

    kit.servo[FAN_IDX].angle = 0

