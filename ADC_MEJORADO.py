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
import requests

def map(x1, in_min, in_max, out_min, out_max):  #function for sensor calibration
    return int((x1-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def volt_chanel(chanel):
    max_val= 26100
    voltage_ref= 3.3
    max_val1= 24808
    voltage_ref1= 5.0
    mult2 = 0.000125
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(3, gain=2/3)
    #volt = adc.read_adc(chanel, gain=2) * mult2 
    v = value / max_val * voltage_ref
    v1 = value / max_val1 * voltage_ref1
    return value,v1
'''

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
    buf = []
    for i in range(10):# Take 10 samples
        buf.append(temperature)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        t=(round(avg,1))
   
    return t

def Turbidity():
    mult2 = 0.000125
    adc = Adafruit_ADS1x15.ADS1115()
    buf = []
    for i in range(10):# Take 10 samples
        volt = adc.read_adc(1, gain=1) * mult2
        buf.append(volt)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        x1=(round(avg,2))
        
    #Ph sensor calibration parameters
    in_min = 0.0 
    in_max = 3.3
    out_min = 0.0
    out_max = 20
    
    Turbidity = map(x1, in_min, in_max, out_min, out_max)
    
    return Turbidity
    

while True:
                  
    print(pH2_0(),WaterTemperature(),Turbidity())  #Imprimimos el resultado
    #write()
    #enviar=requests.get("https://api.thingspeak.com/update?api_key=8BJ1YM0AH5ADVY3X&field1="+str(pH_probe())+"&field2="+str(x)+"&field3="+str(y))
    enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())+"&field2="+str(pH2_0())+"&field3="+str(Turbidity()))
    
    time.sleep(120)
'''
while True:
     print(volt_chanel(3))
     time.sleep(2)
    