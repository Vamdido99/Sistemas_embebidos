#!/usr/bin/python
import smbus2
import bme280
import time                             
from w1thermsensor import W1ThermSensor 
import board
import busio
import numpy as np
import sys
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import Adafruit_ADS1x15
import requests
import datetime
import RPi.GPIO as GPIO

def map(x1, in_min, in_max, out_min, out_max):  #function for sensor calibration
    return int((x1-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def volt_chanel(chanel):
    max_val1= 25845
    voltage_ref1= 5.14
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(chanel, gain=2/3)
    v1 = value / max_val1 * voltage_ref1
    return v1

def Temp_Humidity():
    bus = smbus2.SMBus(1)
    calibration_params = bme280.load_calibration_params(bus, 0x76)
    bme = bme280.sample(bus, 0x76, calibration_params)
    t  = (round(bme.temperature,2))
    pa = (round(bme.pressure,2))
    Hr = (round(bme.humidity,2)) 
    
    return t,pa,Hr

def pH2_0():
    buf = []
    for i in range(10):# Take 10 samples
        volt = volt_chanel(1) 
        buf.append(volt)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        x1=(round(avg,2))
    #Ph sensor calibration parameters
    in_min = 0
    in_max = 3.0
    out_min = 0.0
    out_max = 14
    
    pH20 = map(x1, in_min, in_max, out_min, out_max)
    return pH20

def WaterTemperature():
    sensor = W1ThermSensor() #Create the object sensor
    temperature = sensor.get_temperature()
    buf = []
    for i in range(10):# Take 10 samples
        buf.append(temperature)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        t=(round(avg,1))
    return t

def Turbidity():
    buf = []
    for i in range(10):# Take 10 samples
        volt = volt_chanel(0)
        buf.append(volt)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        x1=(round(avg,2))
        
    #Ph sensor calibration parameters
    in_min = 0.0 
    in_max = 4.5
    out_min = 0.0
    out_max = 20
    
    Turbidity = map(x1, in_min, in_max, out_min, out_max)
    
    return Turbidity

Temp_Humidity_Values = Temp_Humidity()

def write():
    date = datetime.datetime.now().replace(microsecond=0).isoformat()
    f = open("datos.cvc","a")
    f.write(date)
    f.write("  WaterTemperature = ")
    f.write(str(WaterTemperature()))
    f.write("  ")
    f.write("  Ph = ")
    f.write(str(pH2_0()))
    f.write("  ")
    f.write("  Turbidity = ")
    f.write(str(Turbidity()))
    f.write("  ")
    f.write("  Ambient Temperature = ")
    f.write(str(Temp_Humidity_Values[0]))
    f.write("  ")
    f.write("  Atmospheric Pressure = ")
    f.write(str(Temp_Humidity_Values[1]))
    f.write("  ")
    f.write("  Ambient Humidity = ")
    f.write(str(Temp_Humidity_Values[2]))
    f.write("  ")
    f.write("\n")
    f.close()

def toggle_gpio(state,gpio_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_pin, GPIO.OUT)
    if state == "on":
        GPIO.output(gpio_pin, GPIO.HIGH)
        print(" GPIO port on")
    elif state == "off":
        GPIO.output(gpio_pin, GPIO.LOW)
        print("GPIO port off")
    else:
        print("Invalid state, Must be 'on' or 'off'.")
        

    
while True:
    toggle_gpio("on",21)
    time.sleep(40)
    write()
    enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())+"&field2="+str(pH2_0())+"&field3="+str(Turbidity())+"&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
    print(Turbidity(),WaterTemperature(),pH2_0(),Temp_Humidity())
    toggle_gpio("off",21)
    time.sleep(300)
    
