import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import copy
import json
import argparse
import numpy as np
import curses
import warnings
import evdev
from threading import Thread
import board
import busio
import digitalio
import adafruit_tlc59711
from adafruit_servokit import ServoKit
import logging

# argument parser
parser = argparse.ArgumentParser(description="hexa")
parser.add_argument('-c', '--conf', required=True, help='path to json config file needed')
args = vars(parser.parse_args())
print(args)

# deal with warnings and load config
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))

# init global variables
global control
global imu
global poll_interval
global conf
global motors =  conf['motors']

# init logging info
logging.basicConfig(filename='debug.log',level=logging.DEBUG)
logging.debug('LOGGING INFORMATION:
logging.basicConfig(format='[%(levelname)s] (%(threadName)-10s) %(message)s',)

# set up line pointer for curses interface
#global curses_line 0

#define input codes for PS3 controller
ps3_codes = {'l_but':295, 'u_but':292, 'r_but':293, 'd_but':294,
        'x_but':302, 's_but':303, 't_but':300, 'c_but':301,
        'l1':298, 'l2':296, 'r1':299, 'r2':297,
        'start':291, 'sel':288,
        'l_joy':289, 'r_joy':290}
#Note: type 3: analog, type 1: key press

# class used for interacting with shared memory
class Spin_lock:
    def __init__(self, a):
        self.resource = a
        self.locked = False

    def acquire(self):
        while (self.locked):
            pass
        self.locked = True

    def release(self):
        self.locked = False

    def set(self, val):
        self.resource = val

    def get(self):
        return self.resource

class Controller:
    def __init__(self):
        self.x = False
        self.o = False
        self.tri = False
        self.sqr = False
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.start = False
        self.select = False
        self.home = False
        self.l_bump = False
        self.r_bump = False
        self.l_joy = False
        self.r_joy = False
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.l_trig = 0.0
        self.r_trig = 0.0

class IMU:
    def __init__(self):
        self.accel = [0.0,0.0,0.0]
        self.accel_filtered = [0.0,0.0,0.0]
        self.vel = [0.0,0.0,0.0]
        self.pos = [0.0,0.0,0.0]
        self.a_vel = [0.0,0.0,0.0]
        self.a_vel_filtered = [0.0,0.0,0.0]
        self.angle_comp = [0.0,0.0,0.0]
        self.angle_comp = [0.0,0.0,0.0]
        self.angle_fus = [0.0,0.0,0.0]
        self.angle_fus_q = [0.0,0.0,0.0]
        self.time_prev = 0  # microseconds
        self.time_cur = 0
        self.bias = 0.0
        self.deadband = 0.0
        self.calibrated = False

def start_imu(stdscr):
    # set up IMU
    global poll_interval
    SETTINGS_FILE = "RTIMUlib"
    stdscr.addstr("setings file: " + SETTINGS_FILE + ".ini\n")
    if not os.path.exists(SETTINGS_FILE + ".ini"):  # if no file, create one
        stdscr.addstr("file not found, created settings file\n")
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_dev = RTIMU.RTIMU(settings) # creating IMU object
    stdscr.addstr("IMU Name: " + imu_dev.IMUName() + "\n")

    if (not imu_dev.IMUInit()):
        stdscr.addstr("failed to init IMU\n")
        sys.exit(1)
    else:
        stdscr.addstr("successfully initialized IMU\n")
    stdscr.refresh()

    imu_dev.setSlerpPower(0.02)
    imu_dev.setGyroEnable(True)
    imu_dev.setAccelEnable(True)
    imu_dev.setCompassEnable(True)

    poll_interval = imu_dev.IMUGetPollInterval()
    stdscr.addstr("poll interval: %d\n" % poll_interval)
    stdscr.refresh()
    return imu_dev

def init_controller(stdscr):
    stdscr.addstr(1,0,"initializing controller...\n")
    stdscr.refresh()
    conn = False

    while not conn:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            stdscr.addstr(str(device.name)+str(device.path))
            stdscr.refresh()
            #if device.name == 'Sony PLAYSTATION(R)3 Controller':
            # Why did the device change name????????
            if device.name == 'PLAYSTATION(R)3Conteroller-PANHAI':
                dev = device.path
                ps3 = evdev.InputDevice(dev)
                stdscr.addstr(2,0,device.name+' '+device.path)
                stdscr.addstr(3,0,"found ps3 contoller\n")
                stdscr.refresh()
                conn = True

    #loop until start button is pressed
    for event in ps3.read_loop():
        stdscr.addstr(str(event)+'\n')
        stdscr.refresh()
        if event.type == 1 and event.code == 315 and event.value == 1:
            stdscr.erase()
            stdscr.addstr('starting\n')
            stdscr.refresh()
            time.sleep(1)
            return ps3

def read_controller(dev):
    logging.debug('starting')
    global control
    for event in dev.read_loop():
        control.acquire()
        pass    # do nothing for now
        control.release()

def read_imu(dev,stdscr):
    logging.debug('starting')
    global imu
    global poll_interval
    global conf
    size = conf['accel_filter_num']
    accel_hist = np.zeros(size)
    stdscr.addstr('started imu thread...\n')
    stdscr.refresh()
    while True:
        logging.debug('running')
        if dev.IMURead():
           # read data from IMU
           data = dev.getIMUData()

           imu.acquire()
           logging.debug('acquired imu')
           imu.get().a_vel = data['gyro']
           imu.get().angle_fus = data['fusionPose']
           imu.get().angle_fus_q = data['fusionQPose']
           imu.get().accel = data['accel']
           imu.get().comp = data['compass']
           imu.get().time_prev = imu.get().time_cur
           imu.get().time_cur = data['timestamp']
           # print((time_cur - self.last_update)*10**6)

           if imu.get().calibrated:
               # check deadband and account for bias
               if ((imu.get().a_vel < imu.get().bias + imu.get().deadband) or (imu.get().a_vel > imu.get().bias - imu.get().deadband)):
                   imu.get().a_vel_filtered = 0.0
               else:
                   imu.get().a_vel_filtered = imu.get().a_vel - imu.get().bias

               # use median filter for acceleromater readings to help with spikes
               accel_hist = np.roll(accel_hist, 1)
               accel_hist[0] = imu.get().accel
               imu.get().accel_filtered = np.sort(accel_hist)[int(size/2.0)]
               imu.release()

               # update complementary filter
               complementary_filter()
           else:
               imu.release()

           time.sleep(poll_interval * 1.0/1000.0)

def calibrate_imu(num_cal, stdscr):
    global imu
    global poll_interval
    stdscr.addstr('Calibrating IMU: {} measurements\n'.format(num_cal))
    stdscr.refresh()
    bias = [0.0, 0.0, 0.0]
    deadband = [0.0,0.0,0.0]
    min = [0.0, 0.0, 0.0]
    max = [0.0, 0.0, 0.0]

    for j in range (0, num_cal):
        imu.acquire()
        for i, each in enumerate(imu.get().a_vel):
            bias[i] = bias[i] + each
            if each > max[i]:
                max[i] = each
            elif each < min[i]:
                min[i] = each
        imu.release()

        # wait for imu readings to update
        time.sleep(2.0 * poll_interval * 1.0/1000.0)

    bias[:] = [k / num_cal for k in bias]
    deadband[:] = [(max[m] - min[m]) for m in range(len(min))]
    imu.acquire()
    imu.get().bias = bias
    imu.get().deadband = deadband
    imu.get().calibrated = True
    imu.release()
    stdscr.addstr('Calibration Complete.\n')
    stdscr.addstr('bias: {}   deadband: {}\n'.format(bias, deadband))
    stdscr.refresh()
    time.sleep(2)

# display hopefully useful info
# things to add: bias, deadband, accel, vel, etc.
def update_scr(stdscr):
    global imu
    stdscr.erase()
    imu.acquire()
    stdscr.addstr(1,0,'gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().a_vel)))
    stdscr.addstr(2,0,'accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(imu.get().accel))
    stdscr.addstr(3,0,'fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_fus)))
    stdscr.addstr(4,0,'fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_fus_q)))
    stdscr.addstr(5,0,'complem - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_comp)))
    stdscr.addstr(6,0,'time - {}'.format(imu.get().time_cur))
    imu.release()
    stdscr.refresh()

def complementary_filter():
    global imu
    global conf

    tmp_gyro = [0.0,0.0,0.0]
    tmp_acc = [0.0,0.0,0.0]

    # init rgb
    spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
    rgb = adafruit_tlc59711.TLC59711(spi)
    rgb[0] = (65535, 0, 0)
    rgb[1] = (65535, 0, 0)
    rgb[2] = (0, 16000, 16000)
    rgb[3] = (0, 16000, 16000)

    # gyroscope data returns only change in pos
    # p = integral(dp/dt)
    # because this is a discrete case, just sum change times time elapsed
    imu.acquire()
    delta_t = (imu.get().time_cur - imu.get().time_init) / 10**6    # convert from microseconds to seconds
    for i, each in enumerate(tmp_gyro):
        each = imu.get().a_vel * delta_t

    # based on Tilt Sensing Using a Three-Axis Accelerometer by Mark Pedley
    # (http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1)
    tmp_acc[0] = math.atan(acc_raw[1]/acc_raw[2])
    tmp_acc[1] = math.atan(-1*acc_raw[0]/(acc_raw[1]**2+acc_raw[2]**2)**0.5)

    # Note: only pitch and roll are valid from this estimation (for yaw use compass)
    imu.get().angle_comp += conf['gyro_sensitivity'] * np.asarray(tmp_gyro) + (1-conf['gyro_sensitivity']) * np.asarray(tmp_acc)
    imu.release()

def main(stdscr):
    global control
    global imu
    global conf

    # set up terminal output
    curses.use_default_colors()
    stdscr.scrollok(1) #enable scrolling

    # initialize controller
    ps3 = init_controller(stdscr)
    control = Spin_lock(Controller())

    # initialize imu
    imu_dev = start_imu(stdscr)
    imu = Spin_lock(IMU())

    # initialize threads
    control_t = Thread(target=read_controller, args=(ps3,))
    imu_t = Thread(target=read_imu, args=(imu_dev,stdscr,))
    control_t.start()
    imu_t.start()

    time.sleep(0.5) # wait for imu to init...
    calibrate_imu(conf['num_cal'], stdscr)

    while True:
        update_scr(stdscr)
        #time.sleep(0.1)

if __name__ == "__main__":
    curses.wrapper(main)
