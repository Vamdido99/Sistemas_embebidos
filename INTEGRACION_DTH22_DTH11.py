#!/usr/bin/python
import datetime
import board
import busio
import time
import adafruit_dht
import psutil
import sys
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from time import sleep
import requests

def DTH_22():
    flag_DTH22 = False
    avg_temp = 0
    avg_humidity = 0
    
    for proc in psutil.process_iter():
        if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
            proc.kill()
            
    sensor = adafruit_dht.DHT22(board.D23)
    temp_list = []
    humidity_list = []

        
    for i in range(20):
        for j in range(5):
            try:
                temp = sensor.temperature
                humidity = sensor.humidity
              
                
                if temp is not None:
                    temp_list.append(temp)
                    
                if humidity is not None:
                    humidity_list.append(humidity)
                
                temp = None
                humidity = None
                time.sleep(0.5)
               
                break
            
            except RuntimeError as error:
                time.sleep(0.5)
                continue
            
            except Exception as error:
                sensor.exit()
                raise error
                
        else:
            # If we reach this point, we've attempted to read the sensor 5 times without success
            # Log an error message and exit the function
            #print("Error: Could not read the sensor. Exiting function.")
            return None
            
    if all(x is None for x in temp_list):
        
        flag_DTH22 = False
        temp_rounded = 0
        humidity_rounded =0
        
        return flag_DTH22,temp_rounded,humidity_rounded
        
    else:
        
        flag_DTH22 = True
        temp_list.sort()
        humidity_list.sort()
        
        avg_temp = sum(temp_list[6:15]) / len(temp_list[6:15])
        temp_rounded = round(avg_temp, 1)
        
        avg_humidity = sum(humidity_list[6:15]) / len(humidity_list[6:15])
        humidity_rounded = round(avg_humidity, 1)
        
     
    return flag_DTH22,temp_rounded,humidity_rounded


def DTH_11():
    flag_DTH11 = False
    avg_temp = 0
    avg_humidity = 0
    
    for proc in psutil.process_iter():
        if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
            proc.kill()
            
    sensor = adafruit_dht.DHT11(board.D6)
    temp_list = []
    humidity_list = []

        
    for i in range(20):
        for j in range(5):
            try:
                temp = sensor.temperature
                humidity = sensor.humidity   
                
                if temp is not None:
                    temp_list.append(temp)
                    
                temp = None
                humidity = None
                time.sleep(0.5)
                
                break
            
            except RuntimeError as error:
                time.sleep(0.5)
                continue
            
            except Exception as error:
                sensor.exit()
                raise error
                
        else:
            # If we reach this point, we've attempted to read the sensor 5 times without success
            # Log an error message and exit the function
            #print("Error: Could not read the sensor. Exiting function.")
            return None
            
    if all(x is None for x in temp_list):
        
        
        flag_DTH11 = False
        temp_rounded = 0
        humidity_rounded =0
        
        return flag_DTH11,temp_rounded
        
    else:

        flag_DTH11 = True
        temp_list.sort()
        avg_temp = sum(temp_list[6:15]) / len(temp_list[6:15])
        temp_rounded = round(avg_temp, 1)
    
    return flag_DTH11,temp_rounded

def measure_temperature():
    try:
        flag_DTH22, temp_DTH22, humidity_DTH22 = DTH_22()
        flag_DTH11, temp_DTH11 = DTH_11()

        if flag_DTH22 and flag_DTH11:
            sum_temp = (temp_DTH22 + temp_DTH11)
            prom_temp = round((sum_temp / 2),1)
            prom_humidity = humidity_DTH22
            
            return prom_temp,prom_humidity
           
        
        elif flag_DTH22 and (not flag_DTH11):
            prom_temp = temp_DTH22
            prom_humidity = humidity_DTH22
            
            
            return prom_temp,prom_humidity
           
        
        elif (not flag_DTH22) and flag_DTH11:
            prom_temp = temp_DTH11
            prom_humidity = 0
            
            return prom_temp,prom_humidity
        
            
            
        else:
            prom_temp = 0
            prom_humidity = 0
            return prom_temp,prom_humidity

    except Exception as e:
        print(f"An error occurred: {e}")

print(measure_temperature())


