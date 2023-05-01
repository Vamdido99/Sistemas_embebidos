#!/usr/bin/python
''' Raspberry Pi, ADS1115, PH4502C Calibration '''
import board
import busio
import time
import sys
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Setup 
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
channel = AnalogIn(ads, ADS.P0)

def read_voltage(channel):
    while True:
        buf = list()
        
        for i in range(10):# Take 10 samples
            buf.append(channel.voltage)
        buf.sort() # Sort samples and discard highest and lowest
        buf = buf[2:-2]
        avg = (sum(map(float,buf))/6) # Get average value from remaining 6

        #print(round(avg,2),'V')
        a=(round(avg,2))
        return a
        #time.sleep(2)
        
x1 = read_voltage(channel)

in_min = 0.03 
in_max = 3.06
out_min = 0.0
out_max = 14

def map(x1, in_min, in_max, out_min, out_max):
    return int((x1-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)


l = map(x1, in_min, in_max, out_min, out_max)

while True:
    print(l)
    time.sleep(2)
