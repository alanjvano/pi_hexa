import board
import busio
import digitalio
import adafruit_tlc59711
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
rgb = adafruit_tlc59711.TLC59711(spi)
rgb[0] = (65535, 0, 0)
rgb[1] = (65535, 0, 0)
rgb[2] = (0, 16000, 16000)
rgb[3] = (0, 16000, 16000)

rgb.red_brightness = 63
