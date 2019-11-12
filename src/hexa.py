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

    # update object values with data from IMU
    def update(self, data):
        self.gyro_raw = np.degrees(data['gyro'])
        self.pos_fus = np.degrees(data['fusionPose'])
        self.pos_fus_q = np.degrees(data['fusionQPose'])
        self.acc_raw = data['accel']
        self.comp = np.degrees(data['compass'])
    
    # display hopefully useful info
    def disp(self):
        print('gyro    - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.gyro_raw),end='\n')
        print('accel   - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.acc_raw),end='\n')
        print('fusion  - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.pos_fus),end='\n')
        print('fusionq - x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(self.pos_fus_q),end='\r\r\r\r')
        sys.stdout.flush()


def main(stdscr):
    # create hexacopter object
    hexa = hexacopter()

    while True:
        if imu.IMURead():
            # read data from IMU
            data = imu.getIMUData()
            hexa.update(data) 

            time.sleep(poll_interval * 1.0/1000.0)
        hexa.disp();


    
def complementary_filter(gyro_pos, gryo_sensitivity, acc_sensitivity, g_data):
   gyro_c = copy.copy(gyro_pos)

   # simple integration of gryoscope date
   gyro_c[0] += (g_data[0] / gyro_sensitivity) * (poll_interval * 0.001)
   gyro_c[1] += (g_data[1] / gyro_sensitivity) * (poll_interval * 0.001)
   gyro_c[2] += (g_data[2] / gyro_sensitivity) * (poll_interval * 0.001)



if __name__ == "__main__":
    wrapper(main)
