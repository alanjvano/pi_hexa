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
from curses import wrapper
import warnings

# argument parser
parser = argparse.ArgumentParser(description="hexa")
parser.add_argument('-c', '--conf', required=True, help='path to json config file needed')
args = vars(parser.parse_args())
print(args)

# deal with warnings and load config 
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))

#IMU_REFRESH = 
#print("name: " + __name__)

# set up IMU

SETTINGS_FILE = "RTIMUlib"
print("setings file: " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):  # if no file, create one
    print("file not found, created settings file")
settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings) # creating IMU object
print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("failed to init IMU")
    sys.exit(1)
else:
    print("successfully initialized IMU")
    
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("poll interval: %d" % poll_interval)

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

    # update object values with data from IMU
    def update(self):
         if imu.IMURead():
            # read data from IMU
            data = imu.getIMUData()
        
            self.gyro_raw = data['gyro']
            self.pos_fus = data['fusionPose']
            self.pos_fus_q = data['fusionQPose']
            self.acc_raw = data['accel']
            self.comp = data['compass']

            # update complementary filter
            [self.pos_comp, self.last_update] = complementary_filter(self.pos_comp, self.gyro_raw, self.acc_raw,
                    self.gyro_sensitivity, self.acc_sensitivity, self.last_update, self.gyro_bias)
        
            time.sleep(poll_interval * 1.0/1000.0)
    
    # display hopefully useful info
    def disp(self, stdscr):
        stdscr.erase()
        stdscr.addstr(1,0,'gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.gyro_raw)))
        stdscr.addstr(2,0,'accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.acc_raw)))
        stdscr.addstr(3,0,'fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_fus)))
        stdscr.addstr(4,0,'fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_fus_q)))
        stdscr.addstr(5,0,'complem - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(np.degrees(self.pos_comp)))
        stdscr.refresh()

    def gyro_calibrate(self):
        # do nothing 
        test = 1

def main(stdscr):
    # create hexacopter object
    hexa = hexacopter()

    # set terminal output font
    curses.use_default_colors()

    while True:
        hexa.update()
        hexa.disp(stdscr)

    
def complementary_filter(pos_comp, gyro_raw, acc_raw, 
                         gyro_sensitivity, acc_sensitivity, time_init, gyro_bias):
    tmp_gyro = [0,0,0]
    tmp_acc = [0,0,0]

   # gyroscope data returns only change in pos
   # p = integral(dp/dt)
   # because this is a discrete case, just sum change times time elapsed
    delta_t = time.monotonic() - time_init
    for i, each in enumerate(tmp_gyro):
        each = (gyro_raw[i] + gyro_bias[i]) * delta_t
   
    # based on Freesccale Semiconductor Application Note
    # (www.nxp.com/doc/en/application-note/AN3461.pdf)
    tmp_acc[0] = math.atan(acc_raw[1]/acc_raw[2])
    tmp_acc[1] = math.atan(-1*acc_raw[0]/(acc_raw[1]**2+acc_raw[2]**2)**0.5)

    pos_comp += gyro_sensitivity * np.asarray(tmp_gyro) + acc_sensitivity * np.asarray(tmp_acc)

    # return updated position vector and last sample time
    return pos_comp, time.monotonic()


if __name__ == "__main__":
    wrapper(main)
