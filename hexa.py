import sys, getopt
sys.path.append('.')
import RTIMU
import os.path

import time
import math

IMU_REFRESH = 

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

if __name__ == "__main__":
    main()

def main():
    while True:
        if imu.IMURead():
            # read data from IMU
            data = imu.getIMUData()
            print(data)
            
    
