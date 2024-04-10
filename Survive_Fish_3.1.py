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




def volt_chanel(chanel):
    max_val1= 25845
    voltage_ref1= 5
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(chanel, gain=2/3) # value= adc.read_adc(chanel, gain=2/3)
    v1 = value / max_val1 * voltage_ref1
    return value,v1


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
            
            '''
            flag_EC_TDS -> flag indicate state for sensor ec and TDS
            ec_bit --> value in bit of ec, reading for adc 0
            ec ->> value of ec in µS/cm , "electroconductividad"
            TDS --> value of tds in ppm "total dissolved solids"
            '''
        
        
    return flag_EC_TDS,ec_bit,ec,TDS


def read_PH():
    
    flag_PH = False
    adc = Adafruit_ADS1x15.ADS1115()
    PH_SENSOR_PIN = 2


    VREF = 5.0
    max_val1 = 25845
    buff = []
    avg_PH = 0
    pH_ = 0
    
    if scan_devices(flag_TH,flag_ADC)[1]:
            
        for i in range(20):
            for j in range(5):
                    try:
                        value = adc.read_adc(PH_SENSOR_PIN, gain=2/3)
                        buff.append(value)
                        time.sleep(0.2)
                        flag_PH = True
                        
                        break  

                    except Exception:
                       # print("Error al leer la temperatura del sensor DS18B20.")
                        flag_PH = False
                        PH = 0
                        break  # Exit both loops if error occurs
        
        if flag_PH:
            
            buff.sort()
            avg_PH = sum(buff[6:15]) / len(buff[6:15])
            round_PH = int(round(avg_PH, 1))
            
            #y = -0,0011x + 23,355  3 points
            #y = -0,0011x + 22,428  2 points

           # 4.0	16835	16877
           # 7.0	13834	14256
           # 10.0	11621	11714

           
            ### for ph 4.0
            if 16835 <= round_PH <= 16877:
                pH_ = -0.0012*(round_PH) + 24.192 ## for ph 4.0  and 7.0 
                print("for ph 4.0")
             
            
             ### for ph 7.0
            elif 13834 <= round_PH <= 14256:
                pH_ = -0.0011*(round_PH) + 22.428   ## for ph 7.0  and 10.0 
                print("for ph 7.0")
            
            ### for ph 10.0
            elif 11621 <= round_PH <= 11714:
                pH_ =  -0.0012*(round_PH) + 24.192  ## for ph 7.0  and 10.0 
                print("for ph 10.0")

            
            pH_3_points = -0.0011*(round_PH) + 23.355   ## for ph 4.0 , 7.0  and 10.0 
            pH_2_points = -0.0011*(round_PH) + 22.428   ## for ph 4.0  and 7.0 
            pH_2_points_2 = -0.0012*(round_PH) + 24.192 ## for ph 7.0  and 10.0 

            
    return flag_PH,pH_3_points,pH_2_points,pH_2_points_2,pH_,round_PH



def AHT10():
    flag_AHT10 = False
    avg_temp = 0
    avg_humidity = 0

    # Get I2C bus
    bus = smbus.SMBus(1)  # Adjust for Raspberry Pi version if needed

    temp_list = []
    humidity_list = []

    for i in range(20):
        for j in range(5):
            try:
                # Configure sensor
                config = [0x08, 0x00]
                bus.write_i2c_block_data(0x38, 0xE1, config)
                time.sleep(0.3)

                # Measure and retrieve data
                MeasureCmd = [0x33, 0x00]
                bus.write_i2c_block_data(0x38, 0xAC, MeasureCmd)
                time.sleep(0.3)
                data = bus.read_i2c_block_data(0x38, 0x00)

                # Process temperature and humidity data
                temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
                ctemp = ((temp * 200) / 1048576) - 50
                
                tmp = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
                ctmp = int(tmp * 100 / 1048576)
                
                if ctemp is not None:
                    temp_list.append(ctemp)
                    
                if ctmp is not None:
                    humidity_list.append(ctmp)
                
                
                ctemp = None
                ctmp = None
                time.sleep(0.3)
                break

            except Exception as error:
                time.sleep(0.3)
                continue

        else:
            # 5 attempts failed, return None
            temp_rounded = 0
            humidity_rounded = 0
            return flag_AHT10,temp_rounded,humidity_rounded

    if all(x is None for x in temp_list):
        flag_AHT10 = False
        temp_rounded = 0
        humidity_rounded = 0
        return flag_AHT10, temp_rounded, humidity_rounded

    else:
        flag_AHT10 = True
        temp_list.sort()
        humidity_list.sort()

        avg_temp = sum(temp_list[6:15]) / len(temp_list[6:15])
        temp_rounded = round(avg_temp, 1)

        avg_humidity = sum(humidity_list[6:15]) / len(humidity_list[6:15])
        humidity_rounded = round(avg_humidity, 1)
        
        
        return flag_AHT10, temp_rounded, humidity_rounded

    
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
                time.sleep(0.3)
               
                break
            
            except RuntimeError as error:
                time.sleep(0.3)
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
                time.sleep(0.3)
                
                break
            
            except RuntimeError as error:
                time.sleep(0.3)
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


def AMBIENT_TEMPERATURE_HUMIDITY():
    prom_temp = 0
    prom_humidity = 0
    flag_t_Hr = False
    
    flag_DTH22, temp_DTH22, humidity_DTH22 = DTH_22()
    flag_DTH11, temp_DTH11 = DTH_11()
    flag_AHT10, temp_AHT10, humidity_AHT10 = AHT10()
    
    try:
        
        if flag_DTH22 and flag_DTH11 and flag_AHT10:
            sum_temp = (temp_DTH22 + temp_DTH11 + temp_AHT10)
            prom_temp = round(sum_temp / 3, 1)
            prom_humidity = round((humidity_DTH22  + humidity_AHT10) / 2, 1)
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_DTH22 and flag_DTH11:
            sum_temp = (temp_DTH22 + temp_DTH11)
            prom_temp = round(sum_temp / 2, 1)
            prom_humidity = humidity_DTH22
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_DTH22 and flag_AHT10:
            sum_temp = (temp_DTH22 + temp_AHT10)
            prom_temp = round(sum_temp / 2, 1)
            prom_humidity = round((humidity_DTH22  + humidity_AHT10) / 2, 1)
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_DTH11 and flag_AHT10:
            sum_temp = (temp_DTH11 + temp_AHT10)
            prom_temp = round(sum_temp / 2, 1)
            prom_humidity = round((humidity_DTH22  + humidity_AHT10) / 2, 1)
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_DTH22:
            prom_temp = temp_DTH22
            prom_humidity = humidity_DTH22
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_DTH11:
            prom_temp = temp_DTH11
            prom_humidity = 0
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        elif flag_AHT10 :
            prom_temp = temp_AHT10
            prom_humidity = round(humidity_AHT10, 1)
            flag_t_Hr = True
            return flag_t_Hr,prom_temp, prom_humidity

        else:
            prom_temp = 0
            prom_humidity = 0
            flag_t_Hr = False
            return flag_t_Hr,prom_temp, prom_humidity
            
    except Exception as e:
        prom_temp = 0
        prom_humidity = 0
        flag_t_Hr = False
        
        return flag_t_Hr,prom_temp,prom_humidity


def INTERNAL_HUMIDITY_TEMPERATURE():
    
    try:
        bus = smbus2.SMBus(1)  
        calibration_params = bme280.load_calibration_params(bus, 0x76) #76
        bme = bme280.sample(bus, 0x76, calibration_params)
        t  = (round(bme.temperature,2))
        pa = (round(bme.pressure,2))
        Hr = (round(bme.humidity,2))
        return True,t,pa,Hr
    
    except OSError as e:
        # Maneja el error
        if e.errno == 121:
            print("Error IO, función Temp_Humidity()  ")
            return False,0,0,0
            
        else:
            print("El error es de otro tipo")
            return False,0,0,0



def write_and_send3():

    flag_WT,water_temperature = ds18b20_temperature()
    flag_PH,pH_3_points,pH_2_points,pH_2_points_2,pH_,round_PH = read_PH()
    flag_EC_TDS,ec_bit,ec,TDS = read_tds_value()
    flag_internal_Hr_t,internal_t,internal_pressure, internal_Hr = INTERNAL_HUMIDITY_TEMPERATURE()
    flag_Ambient_t_Hr,Ambient_t,Ambient_Hr = AMBIENT_TEMPERATURE_HUMIDITY()
    
    fecha_hora_formateada = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    f = open("datos.cvc","a")
    f.write(fecha_hora_formateada)

    try:
        if flag_WT == True:
            f.write("  WaterTemperature = ")
            f.write(str(water_temperature))
            #enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature()))
            flag_WT = False
            print("WaterTemperature",water_temperature)

        if flag_PH == True:
           # Turbidity_Values = Turbidity()
           # pH_Values = pH2_0()
            f.write("  ")
            f.write("  Ph = ")
            f.write(str(pH_))
            f.write("  ")
            f.write("  Bits = ")
            f.write(str(round_PH))
            f.write("  ")
            flag_PH = False
            print("pH_",pH_)
        
        if flag_EC_TDS == True:
            f.write("  ") 
            f.write("  Conductivity = ")
            f.write(str(ec))
            f.write("  ")
            f.write("  TDS = ")
            f.write(str(TDS))
            f.write("  ")
            f.write("  Bits = ")
            f.write(str(ec_bit))
            f.write("  ")
           # enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0]))
            flag_EC_TDS = False
            print("TDS",TDS)

        if flag_internal_Hr_t == True: 
           # Temp_Humidity_Values = Temp_Humidity()
            f.write("  Internal Temperature = ")
            f.write(str(internal_t))
            f.write("  ")
            f.write("Atmospheric Pressure = ")
            f.write(str(internal_pressure))
            f.write("  ")
            f.write("Internal Humidity = ")
            f.write(str(internal_Hr))
            f.write("  ")
           # enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
            flag_Ambient_t_Hr = False
            print("internal_Hr",internal_Hr)
            

        if flag_Ambient_t_Hr == True:  
            f.write("  Ambient Temperature = ")
            f.write(str(Ambient_t))
            f.write("  ")
            f.write("  Ambient Humidity = ")
            f.write(str(Ambient_Hr))
            f.write("  ")
            flag_Ambient_t_Hr = False
            print("Ambient t,hr ",Ambient_t,Ambient_Hr)

    except Exception as e:
        print(e)
        f.write("    Error:   \n")

    f.close()
    
if __name__ == "__main__":

    flag_TH = False
    flag_ADC = False
    write_and_send3()


