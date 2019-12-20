import threading
import queue
import logging
import time
from concurrent.futures import ThreadPoolExecutor

#q = Queue()

def read_controller():
    logging.info("Controller thread started")
    time.sleep(2)
    logging.info("Controller thread stopped")

def read_imu():
    logging.info("IMU thread started")
    time.sleep(2)
    logging.info("IMU thread stopped")

def read_GPS():
    logging.info("GPS thread started")
    time.sleep(2)
    logging.info("GPS thread stopped")

def main():
    executor = ThreadPoolExecuter(max_workers=4)
    cont = executor.submit(read_controller)
    imu = executor.submit(read_imu)
    gps = executor.submit(read_gps)

if __name__ == '__main__':
        main()
