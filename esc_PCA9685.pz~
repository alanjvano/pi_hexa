import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

print("full throttle")
kit.servo[0].angle = 180
time.sleep(5)
print("minimum throttle")
kit.servo[0].angle = 0
time.sleep(10)
print("sleep done")
kit.servo[0].angle = 5
time.sleep(5)
