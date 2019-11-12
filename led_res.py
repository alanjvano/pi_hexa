import board 
import busio
import digitalio
import adafruit_tlc59711

spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
rgb = adafruit_tlc59711.TLC59711(spi)

rgb.red_brightness = 0
rgb.green_brightness = 0
rgb.blue_brightness = 0
