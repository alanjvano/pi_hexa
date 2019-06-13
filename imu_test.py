import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

# check for settings file, if not creat one
print("settings file: " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("file not found, creating file")

# create imu object - should auto detect sensor?
settings = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(settings)

print("IMU Name: " + imu.IMUName())

# attempt to initialize imu
if (not imu.IMUInit()):
    print("could not init")
    sys.exit(1)
else:
    print("successfully initialized")

# test with built in RTQF filter
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("poll int: %d" % poll_interval)
#print(type(poll_interval))

while True:
    if imu.IMURead():
        # read data from sensor
        data = imu.getIMUData()
        #print(data)
        fusionPose = data["fusionPose"]
        print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
        time.sleep(poll_interval*1.0/1000.0)
