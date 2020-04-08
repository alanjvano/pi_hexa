from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from curses import wrapper

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

servos = [None,None,None,None,None,None]
motors = [2,0,1,15,14,8]
for i in range(6):
    servos[i] = servo.Servo(pca.channels[motors[i]], actuation_range=180, min_pulse=1000, max_pulse=2000)

servo1 = servo.Servo(pca.channels[0], actuation_range=180, min_pulse=1000, max_pulse=2000)

def main(stdscr):
    #Clear screen
    stdscr.clear()

    cont = True
    pwmin = 0
    pwmax = 180
    pwm = 0
    incr = 1

    while cont:

        stdscr.clrtoeol()

        c = stdscr.getch()
        if c == ord('t'):
            pwm = pwmax
        elif c == ord('b'):
            pwm = pwmin
        elif c == ord('q'):
            cont = False
            break
        elif c == ord('w'):
            if pwm <= pwmax-incr:
                pwm += incr
        elif c == ord('s'):
            if pwm >= pwmin+incr:
                pwm -= incr

        stdscr.addstr(str(pwm))
        for i in range(6):
            servos[i].angle = pwm

wrapper(main)

# close curses
#curses.nocbreak()
#stdscr.keypad(False)
#curses.echo()
#curses.endwin()
