from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

for i in range(0, 18):
    kit.servo[0].angle = i*10
    print(i)
    time.sleep(1)
    #kit.servo[0].angle = 30
