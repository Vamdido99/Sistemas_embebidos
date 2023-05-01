
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

    
def Turbidity2():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    chan = AnalogIn(ads, ADS.P0)
    value =chan.voltage
    #print("{:>5}\t{:>5.3f}".format(chan.value, chan.voltage))
    #time.sleep(0.5)
    
    return value

#Turbidity2()
#while True:

x1 = Turbidity2()

in_min = 0.0035 
in_max = 3.042
out_min = 0.0
out_max = 20

def map(x1, in_min, in_max, out_min, out_max):
    return int((x1-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

l = map(x1, in_min, in_max, out_min, out_max)

while True:
    print(l)
    time.sleep(2)


  