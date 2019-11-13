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

# argument parser
parser = argparse.ArgumentParser(description="hexa")
parser.add_argument('-c', '--conf', required=True, help='path to json config file needed')
args = vars(parser.parse_args())
print(args)

# Variables
gyro_pos = [0,0, 0]  # pitch (x), roll (y), yaw(z)
gyro_sensitivity = 65.536
acc_sensitivity = 8192.0

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

    # update object values with data from IMU
    def update(self):
         if imu.IMURead():
            # read data from IMU
            data = imu.getIMUData()
        
            # convert to degrees
            self.gyro_raw = np.degrees(data['gyro'])
            self.pos_fus = np.degrees(data['fusionPose'])
            self.pos_fus_q = np.degrees(data['fusionQPose'])
            self.acc_raw = data['accel']
            self.comp = np.degrees(data['compass'])
        
            time.sleep(poll_interval * 1.0/1000.0)
    
    # display hopefully useful info
    def disp(self, stdscr):
        stdscr.erase()
        stdscr.addstr(1,0,'gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.gyro_raw))
        stdscr.addstr(2,0,'accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.acc_raw))
        stdscr.addstr(3,0,'fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.pos_fus))
        stdscr.addstr(4,0,'fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.pos_fus_q))
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

    
def complementary_filter(gyro_raw, acc_raw, gryo_sensitivity, acc_sensitivity, time_init):
    tmp_pos = [0,0,0]

   # gyroscope data returns only change in pos
   # p = integral(dp/dt)
   # because this is a discrete case, just sum change times time elapsed
    for i in range(len(gyro_raw)):
        tmp_pos += gyro_raw * (time.monotonic - time_init)


if __name__ == "__main__":
    wrapper(main)
