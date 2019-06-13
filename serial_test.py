import serial
import pynmea2

ser = serial.Serial('/dev/serial0', 9600, timeout=1)
while True:
    data = ser.readline()
    print(data)
    msg = pynmea2.parse(data.str())
    print(msg)
