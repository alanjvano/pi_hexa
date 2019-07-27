import time
from adafruit_servokit import ServoKit
from curses import wrapper

# setup curses library
#stdscr = curses.initscr()
#curses.noecho()
#curses.cbreak()
#stdscr.keypad(True)

kit = ServoKit(channels=16)
motors = [0,1,2,8,14,15]

#int("full throttle")
#kit.servo[0].angle = 180
#time.sleep(5)
#print("minimum throttle")
#kit.servo[0].angle = 0
#time.sleep(10)
#print("sleep done")
#kit.servo[0].angle = 5
#time.sleep(5)

#stdscr.scrollok(1)

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
        for i in motors:
            kit.servo[i].angle = pwm

wrapper(main)

# close curses
#curses.nocbreak()
#stdscr.keypad(False)
#curses.echo()
#curses.endwin()
