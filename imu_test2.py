import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
from threading import Thread

global poll_interval

def start_imu():
    # set up IMU
    global poll_interval
    SETTINGS_FILE = "RTIMUlib"
    print("setings file: " + SETTINGS_FILE + ".ini\n")
    if not os.path.exists(SETTINGS_FILE + ".ini"):  # if no file, create one
        print("file not found, created settings file\n")
    settings = RTIMU.Settings(SETTINGS_FILE)
    imu_dev = RTIMU.RTIMU(settings) # creating IMU object
    print("IMU Name: " + imu_dev.IMUName() + "\n")

    if (not imu_dev.IMUInit()):
        print("failed to init IMU\n")
        sys.exit(1)
    else:
        print("successfully initialized IMU\n")

    imu_dev.setSlerpPower(0.02)
    imu_dev.setGyroEnable(True)
    imu_dev.setAccelEnable(True)
    imu_dev.setCompassEnable(True)

    poll_interval = imu_dev.IMUGetPollInterval()
    print("poll interval: %d\n" % poll_interval)
    #logger.info('initialized imu unit: {}, poll_interval = {}'.format(imu_dev.IMUName(), poll_interval))
    return imu_dev

def read_imu(imu_dev):
    global poll_interval
    while True:
        if imu_dev.IMURead():
            # read data from sensor
            data = imu_dev.getIMUData()
            #print(data)
            fusionPose = data["fusionPose"]
            print(data)
            print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            print("")
            time.sleep(poll_interval*1.0/1000.0)

def main():
    global poll_interval
    imu_dev = start_imu()
    try:
        imu_thread = Thread(name='imu_thread', target=read_imu, args=(imu_dev,))
        imu_thread.start()
    except:
        print('failed to start thread')

if __name__ == "__main__":
    main()
