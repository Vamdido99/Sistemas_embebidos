#!/usr/bin/python
import smbus2
import bme280
import time                             
from w1thermsensor import W1ThermSensor
import w1thermsensor
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
import subprocess
import os




def ds18b20_temperature():

    flag_ds18b20 = False  # Assume sensor is present initially
    temp_list = []
    temperature = 0

    # Sensor directory verification
    sensor_dir = '/sys/bus/w1/devices/28-0417b2c82cff'  # Update with your sensor directory

    if not os.path.exists(sensor_dir):
        #print(f"Sensor directory '{sensor_dir}' does not exist. Cannot read temperature.")
        flag_ds18b20 = False
        return flag_ds18b20, 0
    
    else:
        
        flag_ds18b20 = True
        
        for i in range(20):
           
            for j in range(3):
                try:
                    sensor = W1ThermSensor()
                    temperature = sensor.get_temperature()
                    temp_list.append(temperature)
                    break  # Exit inner loop if reading successful

                except Exception:
                    flag_ds18b20 = False
                    temperature = 0
                    break  # Exit both loops if error occurs

        if flag_ds18b20:
            # Calculate average only if sensor reading was successful
            temp_list.sort()
            avg_temp = sum(temp_list[6:15]) / len(temp_list[6:15])
            temperature = round(avg_temp, 1)

    return flag_ds18b20, temperature


def scan_devices(flag_TH,flag_ADC):
    bus = smbus2.SMBus(1)
    #found_devices = []
    for address in range(0x03, 0x78):
        try:
            bus.write_quick(address)
           # found_devices.append('%02X' % address)
            # Verifica si se ha encontrado un dispositivo ADC.
            if address == 0x48:
                flag_ADC = True
            # Verifica si se ha encontrado un dispositivo TH.
            if address == 0x76:
                flag_TH = True
        except IOError:
            pass
                
    return flag_TH,flag_ADC


def read_tds_value():
    
    flag_EC_TDS = False
    adc = Adafruit_ADS1x15.ADS1115()
    TDS_SENSOR_PIN = 0


    average_voltage = 0.0
    VREF = 5.0
    max_val1 = 25845
    buff = []
    
    ec = 0
    TDS = 0
    avg_EC_TDS = 0
    factor = 0.5
    
    if scan_devices(flag_TH,flag_ADC)[1]:
        flagWater_t, water_t = ds18b20_temperature()
        
        if flagWater_t: 
            temperature = water_t
            print("wt",temperature)
        
        else:
            temperature = 25
            
            
        for i in range(20):
            for j in range(5):
                    try:
                        value = adc.read_adc(TDS_SENSOR_PIN, gain=2/3)
                        buff.append(value)
                        time.sleep(0.2)
                        flag_EC_TDS = True
                        
                        break  

                    except Exception:
                       # print("Error al leer la temperatura del sensor DS18B20.")
                        flag_EC_TDS = False
                        EC = 0
                        TDS = 0
                        break  # Exit both loops if error occurs
        
        if flag_EC_TDS:
            
            buff.sort()
            avg_EC_TDS = sum(buff[6:15]) / len(buff[6:15])
            round_EC_TDS = round(avg_EC_TDS, 1)
           # print("buff ec",buff)
                
            ########### change to volts adc value #######
            average_voltage = round_EC_TDS / max_val1  * VREF
           # print("average_voltage",average_voltage)
            ##################################################
            
            ##########################################
            # Calculate compesation coeficient #######
            compensation_coefficient = 1.0 + 0.02 * (temperature - 25.0)  #EC25=ECx/(1+b(Tx-25))compensación
           # print("coficient temp",compensation_coefficient)
            ##########################################
            
            #probe with voltage
            ec_v = (average_voltage / compensation_coefficient )          
            #probe with bits
            ec_bit = int(round_EC_TDS / compensation_coefficient) 
          
           
            if	ec_bit <= 50:
                ec = round(0.038 * ec_bit  - 1.566,1)
                #ec1 = round(0.2361 * ec_bit - 556.84,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1)
                print("ecua 1")

            elif 60 < ec_bit < 3050:
                
                ec = round(0.2034 * ec_bit  - 468.8,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                #ec1 = round(0.2361 * ec_bit - 556.84,1)
                print("ecua 2")

            elif 3051 < ec_bit < 4590:
                #ec = 0.2098 * ec_bit - 494.25
                ec = 0.2098 * ec_bit - 494.25
                #ec1 = 0.2361 * ec_bit - 556.84
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                print("ecua 3")


            elif 4590 < ec_bit < 5900:
                #ec1 = round(0.2948 * ec_bit - 869.98,1)
                ec = round(0.2361 * ec_bit - 556.84,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                print("ecua 4")

            else:
                ec = round(0.2361 * ec_bit - 556.84,1)
                print("ecua 5")
            
            #factor for sweet water , 0.5 
            TDS = factor * ec
            
            #print(ec_bit,"ec en bit")
            #print(ec,"µS/cm  ")
            #print(TDS,"ppm Tds")
            '''
            flag_EC_TDS -> flag indicate state for sensor ec and TDS
            ec_bit --> value in bit of ec, reading for adc 0
            ec ->> value of ec in µS/cm , "electroconductividad"
            TDS --> value of tds in ppm "total dissolved solids"
            '''
        
        
    return flag_EC_TDS,ec_bit,ec,TDS

flag_TH = False
flag_ADC = False
print(read_tds_value())