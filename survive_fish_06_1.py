#!/usr/bin/python
import smbus2
import bme280
import smbus
import time                             
from w1thermsensor import W1ThermSensor
import w1thermsensor
import board
import busio
import sys
import Adafruit_ADS1x15
import requests
import datetime
import RPi.GPIO as GPIO
import subprocess
import os
import adafruit_dht
import psutil

def volt_chanel(chanel):
    max_val1= 25845
    voltage_ref1= 5
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(chanel, gain=2/3) # value= adc.read_adc(chanel, gain=2/3)
    v1 = value / max_val1 * voltage_ref1
    return value,v1

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
    ec_bit = 0
    ec_abs = 0
    TDS = 0
    avg_EC_TDS = 0
    factor = 0.5
    temperature2 = 0
    
    
    if scan_devices(flag_TH,flag_ADC)[1]:
        
        flagWater_t, water_t = ds18b20_temperature()
        
        if flagWater_t: 
            temperature = water_t
            temperature2 = temperature
            print("wt",temperature)
        
        else:
            temperature = 25
            temperature2 = 0
            print("wt = 25")
            
            
        for i in range(20):
            for j in range(5):
                    try:
                        value = adc.read_adc(TDS_SENSOR_PIN, gain=2/3)
                        buff.append(value)
                        #time.sleep(0.2)
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
            #absolute conductivity
            ec_bit_abs = int(round_EC_TDS)
           
            if ec_bit <= 50:
                ec = round(0.038 * ec_bit  - 1.566,1)
                ec_abs = round(0.038 * ec_bit_abs  - 1.566,1)
                #ec1 = round(0.2361 * ec_bit - 556.84,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1)
                #print("ecua 1")

            elif 60 < ec_bit < 3050:
                
                ec = round(0.2034 * ec_bit  - 468.8,1)
                ec_abs = round(0.2034 * ec_bit_abs  - 468.8,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                #ec1 = round(0.2361 * ec_bit - 556.84,1)
                #print("ecua 2")

            elif 3051 < ec_bit < 4590:
                #ec = 0.2098 * ec_bit - 494.25
                ec = round(0.2098 * ec_bit - 494.25,1)
                ec_abs = round(0.2098 * ec_bit_abs - 494.25,1)
                #ec1 = 0.2361 * ec_bit - 556.84
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                #print("ecua 3")


            elif 4590 < ec_bit < 5900:
                #ec1 = round(0.2948 * ec_bit - 869.98,1)
                ec = round(0.2361 * ec_bit - 556.84,1)
                ec_abs = round(0.2361 * ec_bit_abs - 556.84,1)
                #ec2 = round((133.42 * ec_v * ec_v * ec_v - 255.86 * ec_v * ec_v + 857.39 * ec_v),1) 
                #print("ecua 4")

            else:
                ec = round(0.2361 * ec_bit - 556.84,1)
                ec_abs = round(0.2361 * ec_bit_abs - 556.84,1)
                #print("ecua 5")
            
            #factor for sweet water , 0.5
            if ec < 0:
                ec = ec * -1
            
            if ec_abs < 0:
                ec_abs = ec_abs * -1
                
            TDS = factor * ec
            
            '''
            flag_EC_TDS -> flag indicate state for sensor ec and TDS
            ec_bit --> value in bit of ec, reading for adc 0
            ec ->> value of ec in µS/cm , "electroconductividad"
            TDS --> value of tds in ppm "total dissolved solids"
            '''
        
        
    return flag_EC_TDS,ec_bit,ec,ec_abs,TDS,temperature2

def toggle_gpio(state,gpio_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_pin, GPIO.OUT)
    if state == "on":
        GPIO.output(gpio_pin, GPIO.HIGH)
       # print("GPIO port ", gpio_pin, "on")
    elif state == "off":
        GPIO.output(gpio_pin, GPIO.LOW)
        #print("GPIO port ", gpio_pin, "off")
    else:
        print("GPIO port, no encontrado ")
        
        
     

def ds18b20_temperature():

    flag_ds18b20 = False  # Assume sensor is present initially
    temp_list = []
    temperature = 0
    count = 0
    
    # Sensor directory verification
    #sensor_dir = '/sys/bus/w1/devices/28-0417b2c82cff'  # Up
    sensor_dir = '/sys/bus/w1/devices/28-1a0ed44687b1'  # Update with your sensor directory
    
    if  os.path.exists(sensor_dir):
        
        while(count<=80):
            try:
                if  os.path.exists(sensor_dir):
                    sensor = W1ThermSensor()
                    temperature = sensor.get_temperature()
                    temp_list.append(temperature)
                    count = count + 1
                   # print(len(temp_list))
                            
            except Exception:
                #flag_ds18b20 = False
                #temperature = 0
                #break  # Exit both loops if error occurs
                print("Exception wt")
                toggle_gpio("off",16)
                toggle_gpio("off",20)
                time.sleep(2)
                toggle_gpio("on",16)
                toggle_gpio("on",20)
                count = count + 1
                time.sleep(3)
                #break  # Exit the loop if sensor directory does not exist
            if len(temp_list) == 20:
                count = 0
                flag_ds18b20 = True
                break
            
            if count == 80:
                count = 0
                temp_list = []
                flag_ds18b20 = False
                temperature = 0
                break  # Exit both loops if error occurs
                
                
        
        if flag_ds18b20:
            # Calculate average only if sensor reading was successful
            temp_list.sort()
            avg_temp = sum(temp_list[6:15]) / len(temp_list[6:15])
            temperature = round(avg_temp, 1)
            temp_list = []

    return flag_ds18b20, temperature


def read_PH():
    
    flag_PH = False
    adc = Adafruit_ADS1x15.ADS1115()
    PH_SENSOR_PIN = 2


    VREF = 5.0
    max_val1 = 25845
    buff = []
    avg_PH = 0
    pH_ = 0
    pH_3_points = 0
    pH_2_points= 0
    pH_2_points_2= 0
    pH_=0
    round_PH=0
    count2 = 0
    
    if scan_devices(flag_TH,flag_ADC)[1]:
        
        while(count2<=60):     
            try:
                if scan_devices(flag_TH,flag_ADC)[1]:
                    value = adc.read_adc(PH_SENSOR_PIN, gain=2/3)
                    buff.append(value)
                    time.sleep(0.6)
                    count2 = count2 + 1
                 

            except Exception:
               # print("Error al leer la temperatura del sensor DS18B20.")
               # flag_PH = False
               # PH = 0
               # break  # Exit both loops if error occurs
                print("Exception ph")
                toggle_gpio("off",16)
                toggle_gpio("off",20)
                time.sleep(3)
                toggle_gpio("on",16)
                toggle_gpio("on",20)
                count2 = count2 + 1
                time.sleep(3)
                
            if len(buff) == 20:
                count2 = 0
                flag_PH = True
                break
            
            if count2 == 40:
                count2 = 0
                buff = []
                flag_PH = False
                PH = 0
                os.system('sudo reboot')
                break  
            
 
    
    if flag_PH:
        
        buff.sort()
        #print(buff)
        print("volt",volt_chanel(2))
        avg_PH = sum(buff[6:15]) / len(buff[6:15])
        round_PH = int(round(avg_PH, 1))
        
        pH_ = round(-0.0021*(round_PH) + 16.585,1)
        
        if pH_ <= 6.0:
            pH_ =  round(pH_ + 3.5,1)
            print("-1-")
        elif pH_ >= 8.5:
            pH_ =  round(pH_ - 2,1)
            print("-2-")


        '''
        valores = [16835,16877,13834,14256,11621,11714]
        #2valores = [15500,18077    13501,15449   10000,13500 ]    

        distancias = [abs(round_PH - valor) for valor in valores]
        indice_minimo = distancias.index(min(distancias))
        print(indice_minimo)
        
        if (indice_minimo == 0) or (indice_minimo == 1):
            pH_ = round((-0.0012*(round_PH) + 24.192)-9.5,1) ## for ph 4.0  and 7.0 
            #print(" ecuation for ph 4.0")
            print("range 0")
        
        elif (indice_minimo == 2) or (indice_minimo == 3):
            pH_ = round((-0.0011*(round_PH) + 22.428)-9.5,1)  ## for ph 7.0  and 10.0 
            print(" range 1")
            
        elif (indice_minimo == 4) or (indice_minimo == 5):
            pH_ =  round((-0.0012*(round_PH) + 24.192)-9.5,1)  ## for ph 7.0  and 10.0 
            print("range 2")
        
        else:
            pH_ = round((-0.0011*(round_PH) + 23.355)-9.5,1)   ## for ph 4.0 , 7.0  and 10.0 
            print("None range, ecuation for with _3_points")
        
        pH_3_points = -0.0011*(round_PH) + 23.355   ## for ph 4.0 , 7.0  and 10.0 
        pH_2_points = -0.0011*(round_PH) + 22.428   ## for ph 4.0  and 7.0,
        pH_2_points_2 = -0.0012*(round_PH) + 24.192 ## for ph 7.0  and 10.0, best comportation ph 7 

        

        #y = -0,0011x + 23,355  3 points
        #y = -0,0011x + 22,428  2 points

        # 4.0	16835	16877
        # 7.0	13834	14256
        # 10.0	11621	11714

       
        ### for ph 4.0
        # 1 # 16835 <= round_PH <= 16877:
        # 2 # 15500 <= round_PH <= 18077:
        
        if 15500 <= round_PH <= 18077: 
            pH_ = round(-0.0012*(round_PH) + 24.192,1) ## for ph 4.0  and 7.0 
            print(" ecuation for ph 4.0")
         
        
         ### for ph 7.0
        # 1 # elif 13834 <= round_PH <= 14256:
        # 2 # 13501 <= round_PH <= 15449:

        elif 13501 <= round_PH <= 15449: #
            pH_ = round(-0.0011*(round_PH) + 22.428,1)   ## for ph 7.0  and 10.0 
            print("ecuation for ph 7.0")
        
        ### for ph 10.0
        # 1 # elif 11621 <= round_PH <= 11714:
        # 2 # 10000 <= round_PH <= 13500: 

        elif 10000 <= round_PH <= 13500: 
            pH_ =  round(-0.0012*(round_PH) + 24.192,1)  ## for ph 7.0  and 10.0 
            print("ecuation for ph 10.0")
            
        else:
            pH_ = round(-0.0011*(round_PH) + 23.355,1)   ## for ph 4.0 , 7.0  and 10.0 
            print("ecuation for with _3_points")
        
       # pH_3_points = -0.0011*(round_PH) + 23.355   ## for ph 4.0 , 7.0  and 10.0 
       # pH_2_points = -0.0011*(round_PH) + 22.428   ## for ph 4.0  and 7.0,
       # pH_2_points_2 = -0.0012*(round_PH) + 24.192 ## for ph 7.0  and 10.0, best comportation ph 7 

        '''  

    return flag_PH,pH_,round_PH


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


def AMBIENT_TEMPERATURE_HUMIDITY():
    prom_temp = 0
    prom_humidity = 0
    flag_t_Hr = False

    flag_AHT10, temp_AHT10, humidity_AHT10 = AHT10()
    
    try:

        if flag_AHT10 :
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
    flag_TH_1,flag_ADC_1 = scan_devices(flag_TH,flag_ADC)
    
    if flag_TH_1 == True: 
    
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

    else:
        return False,0,0,0
    
 


def write_and_send3():
    max_retries = 5
    retry_count = 0
    flag_thingspeak = False
    internal_t = 0  # Inicializar con un valor predeterminado
    
    while retry_count < max_retries:
        try:
            fecha_hora_formateada = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Cortar la alimentación
            toggle_gpio("off", 16)
            toggle_gpio("off", 20)
            time.sleep(3)  # Esperar 5 segundos
            
            # Restablecer la alimentación
            toggle_gpio("on", 16)
            toggle_gpio("on", 20)
            time.sleep(3)  # Esperar 2 segundos para que los sensores se estabilicen
            
            # Leer los sensores
            Ambient_t = 0
            Ambient_Hr = 0
            flag_EC_TDS, ec_bit, ec, ec_abs, TDS, temperature2 = read_tds_value()
            flag_internal_Hr_t, internal_t, internal_pressure, internal_Hr = INTERNAL_HUMIDITY_TEMPERATURE()
            flag_PH, pH_, round_PH = read_PH()

            print("water_temperature", temperature2)
            print("ec", ec)
            print("TDS", TDS)
            print("internal_t,internal_pressure,internal_Hr", internal_t, internal_pressure, internal_Hr)
            print("pH_", pH_)
            print("WaterTemperature", temperature2)
            print("flag_EC_TDS", flag_EC_TDS)
            print("flag_PH", flag_PH)

            if (temperature2 > 0 and flag_internal_Hr_t and flag_EC_TDS and flag_PH):
                if internal_t >= 70:
                    print("Temperature too high, rebooting...")
                    os.system('sudo reboot')
                    return False
               
                Ambient_t = round(internal_t - 9.76,1)
                Ambient_Hr = round(internal_Hr + 15.4,1)
                
                # Escribir en el archivo CSV
                with open("datos.cvc", "a") as f:
                    f.write(f"{fecha_hora_formateada}  ")
                    f.write(f"Ambient Temperature = {Ambient_t}  ")
                    f.write(f"Ambient Humidity = {Ambient_Hr}  ")
                    f.write(f"WaterTemperature = {temperature2}  ")
                    f.write(f"Internal Temperature = {internal_t}  ")
                    f.write(f"Atmospheric Pressure = {internal_pressure}  ")
                    f.write(f"Internal Humidity = {internal_Hr}  ")
                    f.write(f"Conductivity = {ec}  ")
                    f.write(f"Conductivity absolute = {ec_abs}  ")
                    f.write(f"TDS = {TDS}  ")
                    f.write(f"Bits = {ec_bit}  ")
                    f.write(f"Ph = {pH_}  ")
                    f.write(f"Bits = {round_PH}\n")
                
                # Enviar datos a ThingSpeak
                enviar = requests.get(f"https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1={Ambient_t}&field2={Ambient_Hr}&field3={temperature2}&field4={internal_Hr}&field5={ec}&field6={ec_abs}&field7={TDS}&field8={pH_}")
                
                time.sleep(20)
                
                flag_thingspeak = True
                break  # Salir del bucle si todo fue exitoso
            else:
                print(f"Retry {retry_count + 1}: Conditions not met")
                retry_count += 1
                if retry_count >= max_retries:
                    print("Max retries reached. Rebooting system...")
                    os.system('sudo reboot')
                    return False
                time.sleep(30)  # Esperar 1 minuto antes de reintentar
        
        except Exception as e:
            print(f"Error occurred: {e}")
            retry_count += 1
            if retry_count >= max_retries:
                print("Max retries reached. Rebooting system...")
                os.system('sudo reboot')
                return False
            time.sleep(30)  # Esperar 1 minuto antes de reintentar
    
    # Enviar estado a ThingSpeak
    flag_thingspeak_ = 1 if flag_thingspeak else 0
    enviar2 = requests.get(f"https://api.thingspeak.com/update?api_key=MN1FVC52YVLT9GV0&field1={fecha_hora_formateada}&field2={internal_t}&field3={flag_thingspeak_}")
    
    return flag_thingspeak

# Función principal
if __name__ == "__main__":
    while True:
        flag_TH = False
        flag_ADC = False
        
        success = write_and_send3()
        
        if not success:
            print("Failed to send data after multiple retries. System will reboot.")
        else:
            print("Data sent successfully. Waiting for next cycle.")
        
        time.sleep(300)  # Esperar 5 minutos antes de la próxima ejecución