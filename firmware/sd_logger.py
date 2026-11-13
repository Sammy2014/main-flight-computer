# T06 - SD card module on SPI1 (needs sdcard.py from micropython-lib on the Pico)
# The SD module runs from +5V, so the 3S battery + LM2596 must be connected.
from machine import Pin, SPI
import sdcard, os

spi = SPI(1, baudrate=1_000_000, sck=Pin(10), mosi=Pin(11), miso=Pin(8))
sd = sdcard.SDCard(spi, Pin(12))        # CS = GP12
os.mount(sd, "/sd")

with open("/sd/test.txt", "w") as f:
    f.write("rocket test OK\n")

print(open("/sd/test.txt").read())
print("Files:", os.listdir("/sd"))
# "OSError: no SD card" = no 5 V, module plugged in reversed, or card not FAT32.
