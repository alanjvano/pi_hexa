from threading import Thread
import time
from concurrent.futures import ThreadPoolExecutor

global control
global imu

class Mutex:
    def __init__(self, val):
        self.resource = val
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
        self.y = False
        self.b = False
        self.a = False
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
        self.accel = (0.0, 0.0, 0.0)
        self.vel = (0.0, 0.0, 0.0)
        self.pos = (0.0, 0.0, 0.0)
        self.a_vel = (0.0, 0.0, 0.0)
        self.angle = (0.0, 0.0, 0.0)

def read_controller():
    global control
    print("Controller thread started")
    control.acquire()
    control.get().l_joy = True
    control.release()
    print("Controller thread stopped")

def read_imu():
    global imu
    print("IMU thread started")
    imu.acquire()
    imu.get().accel = (1.0, 3.14159, 2.7)
    imu.release()
    print("IMU thread stopped")

def read_gps():
    print("GPS thread started")
    time.sleep(2)
    print("GPS thread stopped")

def main():
    global control
    global imu

    control = Mutex(Controller())
    imu = Mutex(IMU())

    cont = Thread(target=read_controller)
    imu_t = Thread(target=read_imu)
    gps = Thread(target=read_gps)
    cont.start()
    imu_t.start()
    gps.start()

    cont.join()
    imu_t.join()
    gps.join()

    control.acquire()
    print("Control left stick: " + str(control.get().l_joy))
    control.release()



if __name__ == '__main__':
        main()
