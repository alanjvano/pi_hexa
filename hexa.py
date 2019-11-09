import sys, getopt
sys.path.append('.')
import RTIMU
import os.path

import time
import math
import copy

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

def main():
    while True:
        if imu.IMURead():
            # read data from IMU
            data = imu.getIMUData()
            gyro_data = data["gyro"]
            time_cpu = time.time()
            
            # use built in fusion kalman filter from imu library to estimate position of drone
            fusion_filter = data['fusionPose']
            gyro_pos[0] = math.degrees(fusion_filter[0])
            gyro_pos[1] = math.degrees(fusion_filter[1])
            gyro_pos[2] = math.degrees(fusion_filter[2])       
    

            time.sleep(poll_interval * 1.0/1000.0) 

        print('x: {0[0]:.2f}  y: {0[1]:.2f}  z: {0[2]:.2f}'.format(gyro_pos),end='\r')
        sys.stdout.flush()


    
def complementary_filter(gyro_pos, gryo_sensitivity, acc_sensitivity, g_data):
   gyro_c = copy.copy(gyro_pos)

   # simple integration of gryoscope date
   gyro_c[0] += (g_data[0] / gyro_sensitivity) * (poll_interval * 0.001)
   gyro_c[1] += (g_data[1] / gyro_sensitivity) * (poll_interval * 0.001)
   gyro_c[2] += (g_data[2] / gyro_sensitivity) * (poll_interval * 0.001)



if __name__ == "__main__":
    main()
