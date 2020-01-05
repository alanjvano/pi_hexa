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

# init global variables
global control
global imu
global conf
global motors

# argument parser
parser = argparse.ArgumentParser(description="hexa")
parser.add_argument('-c', '--conf', required=True, help='path to json config file needed')
args = vars(parser.parse_args())
print(args)

# deal with warnings and load config
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))

motors = conf['motors']

#logging.debug('LOGGING INFORMATION:')

# set up line pointer for curses interface
#global curses_line 0

# initialize handler for logging to file
def init_logging():
    # init logging info
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] (%(threadName)-15s) %(message)s')
    #logger = logging.getLogger(__name__)
    #logger.setLevel(logging.DEBUG)

    # create the handler
    handler = logging.FileHandler('debug.log', mode='w')
    #handler.setLevel(logging.DEBUG)
    handler.setFormatter(formatter)

    # add handler to logger
    logging.getLogger().addHandler(handler)
    logging.getLogger().setLevel(logging.DEBUG)

    return logging

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
        self.state = {
            'x': False,
            'o': False,
            'tri': False,
            'sqr': False,
            'up': False,
            'down': False,
            'left': False,
            'right': False,
            'start': False,
            'sel': False,
            'home': False,
            'l_bump': False,
            'r_bump': False,
            'l_joy': False,
            'r_joy': False,
            'left_x': 0.0,
            'left_y': 0.0,
            'right_x': 0.0,
            'right_y': 0.0,
            'l_trig': 0.0,
            'r_trig': 0.0
        }

class IMU:
    def __init__(self):
        self.accel = np.array([0.0,0.0,0.0])
        self.accel_filtered = np.array([0.0,0.0,0.0])
        self.vel = np.array([0.0,0.0,0.0])
        self.pos = np.array([0.0,0.0,0.0])
        self.a_vel = np.array([0.0,0.0,0.0])
        self.a_vel_filtered = np.array([0.0,0.0,0.0])
        self.angle_comp = np.array([0.0,0.0,0.0])
        self.angle_comp = np.array([0.0,0.0,0.0])
        self.angle_fus = np.array([0.0,0.0,0.0])
        self.angle_fus_q = np.array([0.0,0.0,0.0])
        self.time_prev = 0  # microseconds
        self.time_cur = 0
        self.bias = 0.0
        self.deadband = 0.0
        self.calibrated = False

def init_controller(stdscr, logger):

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
                logger.info('found controller: {} - {}'.format(device.name, device.path))
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

def read_controller(dev,logger):
    logger.debug('starting')
    global control

    ps3_codes = {
        304: 'x',
        305: 'o',
        315: 'start'
    }

    while True:
        for event in dev.read_loop():
            control.acquire()
            #logger.debug('acquired control')
            if event.code != 0 and event.code != 4:
                control.get().state[ps3_codes[event.code]] = bool(event.value)
            control.release()
            #logger.debug('released control')

def read_imu(stdscr, logger, poll_interval):
    global imu
    global conf

    logger.debug('starting')

    # initialize imu
    SETTINGS_FILE = "RTIMUlib"
    stdscr.addstr("settings file: " + SETTINGS_FILE + ".ini\n")
    logger.info("settings file: " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):  # if no file, create one
        stdscr.addstr("file not found, created settings file\n")
        logger.info("file not found, created settings file")
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_dev = RTIMU.RTIMU(settings) # creating IMU object
    stdscr.addstr("IMU Name: " + imu_dev.IMUName() + "\n")

    if (not imu_dev.IMUInit()):
        stdscr.addstr("failed to init IMU\n")
        logger.info('failed to init IMU, closing thread')
        sys.exit(1)
    else:
        stdscr.addstr("successfully initialized IMU\n")
        logger.info('successfully initialized IMU')
    stdscr.refresh()

    imu_dev.setSlerpPower(0.02)
    imu_dev.setGyroEnable(True)
    imu_dev.setAccelEnable(True)
    imu_dev.setCompassEnable(True)

    #poll_interval = imu_dev.IMUGetPollInterval()
    stdscr.addstr("poll interval: %d\n" % poll_interval)
    stdscr.refresh()
    logger.info('initialized imu unit: {}, poll_interval = {}'.format(imu_dev.IMUName(), poll_interval))

    size = conf['accel_filter_num']
    accel_hist = np.zeros((3, size))

    while True:
        #logging.debug('running')
        if imu_dev.IMURead():
            #logger.debug('reading imu')

            # read data from IMU
            data = imu_dev.getIMUData()

            imu.acquire()
            #logger.debug('acquired imu')
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
                for i, each in enumerate(imu.get().a_vel):
                    if ((each < (imu.get().bias[i] + imu.get().deadband[i])) or (each > (imu.get().bias[i] - imu.get().deadband[i]))):
                        imu.get().a_vel_filtered[i] = 0.0
                    else:
                        imu.get().a_vel_filtered[i] = each - imu.get().bias[i]

                # use median filter for acceleromater readings to help with spikes
                for i, each in enumerate(accel_hist):
                    each = np.roll(each, 1)
                    each[0] = imu.get().accel[i]
                    imu.get().accel_filtered[i] = np.sort(each)[int(size/2.0)]
                imu.release()
                #logger.debug('released imu')

                # update complementary filter
                complementary_filter(logger)
            else:
                imu.release()
                #logger.debug('released imu')

            time.sleep(poll_interval * 1.0/1000.0)

def calibrate_imu(stdscr, num_cal, logger, poll_interval):
    global imu

    stdscr.addstr('Calibrating IMU: {} measurements\n'.format(num_cal))
    stdscr.refresh()
    bias = [0.0, 0.0, 0.0]
    deadband = [0.0,0.0,0.0]
    min = [0.0, 0.0, 0.0]
    max = [0.0, 0.0, 0.0]

    for j in range (0, num_cal):
        imu.acquire()
        logger.debug('acquired imu (calibrate)')
        for i, each in enumerate(imu.get().a_vel):
            bias[i] = bias[i] + each
            if each > max[i]:
                max[i] = each
            elif each < min[i]:
                min[i] = each
        imu.release()
        logger.debug('released imu (calibrate)')

        # wait for imu readings to update
        time.sleep(10.0 * poll_interval * 1.0/1000.0)

    bias[:] = [k / num_cal for k in bias]
    deadband[:] = [(max[m] - min[m]) for m in range(len(min))]
    imu.acquire()
    logger.debug('acquired imu (set bias and deadband)')
    imu.get().bias = bias
    imu.get().deadband = deadband
    imu.get().calibrated = True
    imu.release()
    logger.debug('released imu (set bias and deadband)')
    stdscr.addstr('Calibration Complete.\n')
    stdscr.addstr('bias: {}   deadband: {}\n'.format(bias, deadband))
    stdscr.refresh()
    time.sleep(2)

    logger.info('calibrated imu: deadband={}, bias={}'.format(deadband, bias))

# display hopefully useful info
# things to add: bias, deadband, accel, vel, etc.
def update_scr(stdscr):
    global imu
    global control
    stdscr.erase()
    imu.acquire()
    control.acquire()
    #logger.debug('acquired imu')

    stdscr.addstr(1,0,'gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().a_vel_filtered)))
    stdscr.addstr(2,0,'accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(imu.get().accel_filtered))
    stdscr.addstr(3,0,'fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_fus)))
    stdscr.addstr(4,0,'fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_fus_q)))
    stdscr.addstr(5,0,'comp    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(imu.get().angle_comp)))
    stdscr.addstr(6,0,'time - {}'.format(imu.get().time_cur))
    stdscr.addstr(7,0,'x - {}   0 - {}'.format(control.get().state['x'], control.get().state['o']))
    stdscr.addstr()

    imu.release()
    control.release()
    #logger.debug('released imu')
    stdscr.refresh()

def complementary_filter(logger):
    global imu
    global conf

    tmp_gyro = np.array([0.0,0.0,0.0])
    tmp_acc = np.array([0.0,0.0,0.0])

    # gyroscope data returns only change in pos
    # p = integral(dp/dt)
    # because this is a discrete case, just sum change times time elapsed
    imu.acquire()
    #logger.debug('acquired imu')
    delta_t = (imu.get().time_cur - imu.get().time_prev) / 10**6    # convert from microseconds to seconds
    for i, each in enumerate(tmp_gyro):
        each = imu.get().a_vel[i] * delta_t

    # based on Tilt Sensing Using a Three-Axis Accelerometer by Mark Pedley
    # (http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1)
    tmp_acc[0] = math.atan(imu.get().accel_filtered[1]/imu.get().accel_filtered[2])
    tmp_acc[1] = math.atan(-1 * imu.get().accel_filtered[0] / (( (imu.get().accel_filtered[1]**2) + (imu.get().accel_filtered[2]**2))**0.5) )

    # Note: only pitch and roll are valid from this estimation (for yaw use compass)
    imu.get().angle_comp += conf['gyro_sensitivity'] * tmp_gyro + (1-conf['gyro_sensitivity']) * tmp_acc
    imu.release()

def main(stdscr):
    global control
    global imu
    global conf

    # set up terminal output
    curses.use_default_colors()
    stdscr.scrollok(1) #enable scrolling

    # init logger
    logger = init_logging()

    # initialize controller and imu
    ps3 = init_controller(stdscr, logger)
    control = Spin_lock(Controller())
    imu = Spin_lock(IMU())

    # initialize threads
    try:
        control_t = Thread(name='contr_thread', target=read_controller, args=(ps3,logger,))
        imu_t = Thread(name='imu_thread', target=read_imu, args=(stdscr,logger, conf['poll_int']))
        #control_t.setDaemon(true)
        #imu_t.setDaemon(true)
        control_t.start()
        imu_t.start()
    except:
        logger.debug('threads failed to start')

    time.sleep(0.5) # wait for imu to init...
    calibrate_imu(stdscr, conf['num_cal'], logger, conf['poll_int'])

    # init rgb
    spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
    rgb = adafruit_tlc59711.TLC59711(spi)
    rgb[0] = (65535, 0, 0)
    rgb[1] = (65535, 0, 0)
    rgb[2] = (0, 16000, 16000)
    rgb[3] = (0, 16000, 16000)

    while True:
        update_scr(stdscr)
        time.sleep(0.1)

if __name__ == "__main__":
    curses.wrapper(main)
