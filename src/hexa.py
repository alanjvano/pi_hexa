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

# argument parser
parser = argparse.ArgumentParser(description="hexa")
parser.add_argument('-c', '--conf', required=True, help='path to json config file needed')
args = vars(parser.parse_args())
print(args)

# deal with warnings and load config
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))

# set up line pointer for curses interface
#global curses_line 0

# init global variables
global control
global imu

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
        self.resource = self.a
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
    def __init__(self, dev):
        self.device = dev
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
        self.vel = [0.0,0.0,0.0]
        self.pos = [0.0,0.0,0.0]
        self.a_vel = [0.0,0.0,0.0]
        self.angle = [0.0,0.0,0.0]
        self.comp = [0.0,0.0,0.0]
        self.angle_fus = [0.0,0.0,0.0]
        self.angle_fus_q = [0.0,0.0,0.0]
        self.time = 0.0
        self.bias = 0.0
        self.deadband = 0.0
        self.calibrated = False

def start_imu():
    # set up IMU
    SETTINGS_FILE = "RTIMUlib"
    print("setings file: " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):  # if no file, create one
        print("file not found, created settings file")
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_dev = RTIMU.RTIMU(settings) # creating IMU object
    print("IMU Name: " + imu_dev.IMUName())

    if (not imu_dev.IMUInit()):
        print("failed to init IMU")
        sys.exit(1)
    else:
        print("successfully initialized IMU")

    imu_dev.setSlerpPower(0.02)
    imu_dev.setGyroEnable(True)
    imu_dev.setAccelEnable(True)
    imu_dev.setCompassEnable(True)

    poll_interval = imu_dev.IMUGetPollInterval()
    print("poll interval: %d" % poll_interval)
    return imu_dev

def read_controller(dev):
    global control
    for event in dev.read_loop():
        control.acquire()
        if event.type == 1 and event.code == 315 and event.value == 1:
        control.release()

def read_imu(dev):
    global imu
    while True:
        if dev.IMURead():
           # read data from IMU
           data = dev.getIMUData()

           imu.acquire()
           imu.get().a_vel = data['gyro']
           imu.get().angle_fus = data['fusionPose']
           imu.get().angle_fus_q = data['fusionQPose']
           imu.get().accel = data['accel']
           imu.get().comp = data['compass']
           imu.get().time = data['timestamp']
           # print((time_cur - self.last_update)*10**6)
           imu.release()

           if imu.get().calibrated:
               # update complementary filter
               #[self.pos_comp, self.last_update] = complementary_filter(self.pos_comp, self.gyro_raw, self.acc_raw,
               #        self.gyro_sensitivity, self.acc_sensitivity, self.last_update, time_cur, self.gyro_bias)

           time.sleep(poll_interval * 1.0/1000.0)

def calibrate_imu(dev, num_cal):
    global imu
    bias = [0.0, 0.0, 0.0]
    min = [0.0, 0.0, 0.0]
    max = [0.0, 0.0, 0.0]
    for i in range (0, num_cal):
        imu.acquire()
        for i, each in enumerate(imu.get().a_vel):
            bias[i] = bias[i] + each
            if each > max[i]:
                max[i] = each
            elif each < min[i]:
                min[i] = each
        imu.release()
    imu.acquire()
    imu.get().bias = bias / num_cal
    imu.get().deadband = max - min
    imu.get().calibrated = True
    imu.release()

    # Checking the deadband
    # if (val < bias + deadband) || (val > bias - deadbabd):
    #   return 0
    # else:
    #   return val - bias


class hexacopter:
    def __init__(self):
        self.gyro_raw = [0,0,0]     # raw gyroscope rates
        self.acc_raw = [0,0,0]      # raw accelerometer rates
        self.pos_fus = [0,0,0]      # fusion position
        self.pos_fus_q = [0,0,0]    # fusion position (quaternarion?)
        self.pos_comp = [0,0,0]     # complementary filter pos
        self.comp = [0,0,0]         # compass pos
        self.last_update = time.monotonic()      # last update time
        self.gyro_sensitivity = float(conf["gyro_sensitivity"])
        self.acc_sensitivity = 1 - self.gyro_sensitivity

        # calibrated values
        self.gyro_bias = [0,0,0]

    # display hopefully useful info
    def disp(self, stdscr):
        stdscr.erase()
        stdscr.addstr(1,0,'gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.gyro_raw)))
        stdscr.addstr(2,0,'accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.acc_raw))
        stdscr.addstr(3,0,'fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_fus)))
        stdscr.addstr(4,0,'fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_fus_q)))
        stdscr.addstr(5,0,'complem - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_comp)))
        stdscr.refresh()

    def gyro_calibrate(self):


def main(stdscr):
    global control
    global imu

    # set up terminal output
    curses.use_default_colors()
    stdscr.scrollok(1) #enable scrolling

    # initialize controller
    ps3 = init_controller(stdscr)
    control = Spin_lock(Controller())

    # initialize imu
    imu_dev = start_imu()
    imu = Spin_lock(IMU())

    # initialize threads
    control_t = Thread(target=read_controller(ps3))
    imu_t = Thread(target=read_imu(imu_dev))
    control_t.start()
    imu_t.start()

    calibrate_imu()

def complementary_filter(pos_comp, gyro_raw, acc_raw,
                         gyro_sensitivity, acc_sensitivity, time_init, time_cur, gyro_bias):
    tmp_gyro = [0,0,0]
    tmp_acc = [0,0,0]

   # gyroscope data returns only change in pos
   # p = integral(dp/dt)
   # because this is a discrete case, just sum change times time elapsed
    delta_t = (time_cur - time_init) * 10**6    # convert from microseconds to seconds
    for i, each in enumerate(tmp_gyro):
        each = (gyro_raw[i] + gyro_bias[i]) * delta_t

    # based on Freesccale Semiconductor Application Note
    # (www.nxp.com/doc/en/application-note/AN3461.pdf)
    tmp_acc[0] = math.atan(acc_raw[1]/acc_raw[2])
    tmp_acc[1] = math.atan(-1*acc_raw[0]/(acc_raw[1]**2+acc_raw[2]**2)**0.5)

    pos_comp += gyro_sensitivity * np.asarray(tmp_gyro) + acc_sensitivity * np.asarray(tmp_acc)

    # return updated position vector and last sample time
    return pos_comp, time_cur

def init_controller(stdscr):
    stdscr.addstr(1,0,"initializing controller...")
    stdscr.refresh()
    conn = False

    while not conn:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            #print(device.name, device.path)
            if device.name == 'Sony PLAYSTATION(R)3 Controller':
                dev = device.path
                ps3 = evdev.InputDevice(dev)
                stdscr.addstr(2,0,device.name+' '+device.path)
                stdscr.addstr(3,0,"found ps3 contoller")
                stdscr.refresh()
                conn = True

    #loop until start button is pressed
    for event in ps3.read_loop():
        stdscr.addstr(str(event)+'\n')
        stdscr.refresh()
        if event.type == 1 and event.code == 315 and event.value == 1:
            stdscr.erase()
            stdscr.addstr('starting')
            stdscr.refresh()
            time.sleep(1)
            return ps3


if __name__ == "__main__":
    curses.wrapper(main)
