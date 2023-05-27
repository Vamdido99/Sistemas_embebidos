import smbus2
import bme280
import time

def Temp_Humidity():
   
    bus = smbus2.SMBus(1)
    calibration_params = bme280.load_calibration_params(bus, 0x76)
    bme = bme280.sample(bus, 0x76, calibration_params)
    t  = bme.temperature
    pa = bme.pressure
    Hr = bme.humidity
    
    return t,pa,Hr
  
while True:
    
    print(Temp_Humidity())
    time.sleep(2)
    