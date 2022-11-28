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



def pH_probe():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    channel = AnalogIn(ads, ADS.P0)
    A0 = channel.voltage
    pH = float(-10.641509)*float((A0)) + float(22.55509299)
    time.sleep(1)
    a = format(pH, '.2f')
    return a


def Temperature_Humidity():
    for proc in psutil.process_iter():
        if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
            proc.kill()
    sensor = adafruit_dht.DHT11(board.D14)
    for i in range(3):
        try:
            temp = sensor.temperature
            humidity = sensor.humidity
            a = ("Temperature: {}*C Humidity: {}% ".format(temp, humidity))
        except RuntimeError as error:
            time.sleep(2.0)
            continue
        except Exception as error:
            sensor.exit()
            raise error
        
        time.sleep(2.0)
    
    return  a

def map():
    for proc in psutil.process_iter():
        if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
            proc.kill()
    sensor = adafruit_dht.DHT11(board.D14)
    for i in range(3):
        try:
            temp = sensor.temperature
            humidity = sensor.humidity
            a = (temp, humidity)
        except RuntimeError as error:
            time.sleep(2.0)
            continue
        except Exception as error:
            sensor.exit()
            raise error
        
        time.sleep(2.0)
    
    return  a

x,y =map()


def write():
    date = datetime.datetime.now().replace(microsecond=0).isoformat()
    f = open("datos.cvc","a")
    f.write(date)
    f.write("  pH = ")
    f.write(pH_probe())
    f.write("  ")
    f.write(Temperature_Humidity())
    f.write("\n")
    f.close()

while True:
    write()
    enviar=requests.get("https://api.thingspeak.com/update?api_key=8BJ1YM0AH5ADVY3X&field1="+str(pH_probe())+"&field2="+str(x)+"&field3="+str(y))
    time.sleep(25)