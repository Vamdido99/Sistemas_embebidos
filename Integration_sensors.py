#!/usr/bin/python
import time                             
from w1thermsensor import W1ThermSensor 
import board
import busio
import numpy as np
import time
import sys
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import Adafruit_ADS1x15


def map(x1, in_min, in_max, out_min, out_max):  #function for sensor calibration
    return int((x1-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def Volt_Chanel(chanel):
    
    mult2 = 0.000125
    adc = Adafruit_ADS1x15.ADS1115()
    #value= adc.read_adc(0, gain=1)
    volt = adc.read_adc(chanel, gain=1) * mult2
    return volt


def pH2_0():
    mult2 = 0.000125
    adc = Adafruit_ADS1x15.ADS1115()
    
    buf = []
    for i in range(10):# Take 10 samples
        volt = adc.read_adc(0, gain=1) * mult2
        buf.append(volt)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        x1=(round(avg,2))
        
    #Ph sensor calibration parameters
    in_min = 0.03 
    in_max = 3.03
    out_min = 0.0
    out_max = 14
    
    pH20 = map(x1, in_min, in_max, out_min, out_max)
    
    return pH20
    

#Creamos el objeto sensor
def WaterTemperature():
    sensor = W1ThermSensor() #Creamos el objeto sensor
    temperature = sensor.get_temperature()
    return temperature




#while True:
                   #Obtenemos la temperatura en centígrados
print(pH2_0())  #Imprimimos el resultado
    
   # Ph()
#time.sleep(1)                                         #Esperamos un segundo antes de t
