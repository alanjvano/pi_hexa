from Adafruit_PWM_Servo_Driver import PWM
import time
import curses

pwm = PWM(0x40, debug=True)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz

screen = curses.initscr()
# turn off input echoing
curses.noecho()
# respond to keys immediately (don't wait for enter)
curses.cbreak()
# map arrow keys to special values
screen.keypad(True)

done=False
numbmax = 600
numbmin = 200
numb = numbmin
inc = 10

try:
    while not done:
        char = screen.getch()
        if char == ord('q'):
            done=True
        else:
            if char == curses.KEY_UP:
                if numb < numbmax:
                    numb += inc
                elif char == curses.KEY_DOWN:
                    if numb > numbmin:  
                        numb -= inc
                elif char == ord('t'):
                    numb = numbmax
                elif char == ord('b'):
                    numb = numbmin
        pwm.setPWM(0,0,numb)

finally:
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
    print(numb)
