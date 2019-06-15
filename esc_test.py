import time
#from adafruit_servokit import ServoKit
import adafruit_servokit
#kit = ServoKit(channels=16)

ESC = 0

#kit.servo[ESC].set_pulse_width_range(1000,2000)
adafruit_servokit.pwm.set_pwm_freq(50)

for i in range(41, 83, 2):
    print("throttle: " + str(i))
    #kit.servo[ESC].angle = i
    #time.sleep(1)

    pwm.set_pwm(0, 0, i)    

