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



def volt_chanel(chanel):
    max_val1= 25845
    voltage_ref1= 5.14
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(chanel, gain=2/3) # value= adc.read_adc(chanel, gain=2/3)
    v1 = value / max_val1 * voltage_ref1
    return value

def Temp_Humidity():
    
    try:
    
        bus = smbus2.SMBus(1)  
        calibration_params = bme280.load_calibration_params(bus, 0x76) #76
        bme = bme280.sample(bus, 0x76, calibration_params)
        t  = (round(bme.temperature,2))
        pa = (round(bme.pressure,2))
        Hr = (round(bme.humidity,2))
        return t,pa,Hr,True
    
    except OSError as e:
        # Maneja el error
        if e.errno == 121:
            print("Error IO, función Temp_Humidity()  ")
            return 0,0,0,False
            
           
        else:
            print("El error es de otro tipo")
            # ...
    
   

def pH2_0():
    buf = []
    m = 0.002
    b = -14.973
    m1 = 0.0023
    b1 = -17.669
    
   
    #for i in range(10):# Take 10 samples
    volt = volt_chanel(2)          
     #   buf.append(volt)
      #  buf.sort() # Sort samples and discard highest and lowest
      #  avg = np.average(buf)
       # x1=(round(avg,2))
    #Ph sensor calibration parameters
    pH = (m * volt) + b
    pH20=(round(pH,2))
    return pH20,volt

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
    m = 0.00002
    m2 = 0.5265
    m3 =  4395.8
    
    volt = volt_chanel(0)
        
    #turbidity sensor calibration parameters
    Turbidity = ((m) * volt**2) -(m2 * volt) + m3
    return Turbidity,volt


def Turbidity2():
    buf = []
    for i in range(10):# Take 10 samples
        volt = volt_chanel(0)
        buf.append(volt)
        buf.sort() # Sort samples and discard highest and lowest
        avg = np.average(buf)
        x1=(round(avg,2))
        
    #Ph sensor calibration parameters
    NTU = (x1/4.3)*1000
    NTU2 = 1000-(x1/4.3)*1000
    return NTU2,x1,volt


def toggle_gpio(state,gpio_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_pin, GPIO.OUT)
    if state == "on":
        GPIO.output(gpio_pin, GPIO.HIGH)
        print("GPIO port ", gpio_pin, "on")
    elif state == "off":
        GPIO.output(gpio_pin, GPIO.LOW)
        print("GPIO port ", gpio_pin, "off")
    else:
        print("GPIO port, no encontrado ")

'''
######################
turbidez del agua
#####################

sucia  :  (0.14040781582511122, 706)



limpia :   (4.261357322499516, 21427)




alcohol : (4.313463339137163, 21689)

######################
pH del agua 
#####################


10.00  :(2.4117926097891274, 12127)






7.00 :  (2.1834807506287484, 10979)







4.01  :  (1.8897380537821629, 9502)



'''

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



def get_temperature_and_check_threshold(flag_WT):
    # Obtén el valor de la temperatura
    raw_temperature = subprocess.check_output(["cat", "/sys/bus/w1/devices/28-0417b2c82cff/w1_slave"], text=True).split()[-1]

    # Extrae el valor de la temperatura de la cadena
    temperature = float(raw_temperature[2:])

    # Si la temperatura es igual a t=85000, establece la bandera en falso
    if temperature == 85000:
        flag_WT = False

    # Si la temperatura es diferente de t=85000, establece la bandera en true
    else:
        flag_WT = True

    return flag_WT


 
        
    
    
def write2(flag_ADC, flag_TH, flag_WT):
    print(flag_ADC, flag_TH, flag_WT)
    
    fecha_hora_formateada = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    f = open("datos.cvc","a")
    f.write(fecha_hora_formateada)
    
   # try:
    if flag_WT == True:
        f.write("  WaterTemperature = ")
        f.write(str(WaterTemperature()))
        #enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature()))
       # enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature()))
        
        
    if flag_ADC == True:
        Turbidity_Values = Turbidity()
        pH_Values = pH2_0()
        f.write("  ")
        f.write("  Ph = ")
        f.write(str(pH2_0()))
        f.write("  ")
        f.write("  Turbidity = ")
        f.write(str(Turbidity()))
        f.write("  ")
       # enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0]))
        
             
    if  (flag_TH == True  and  Temp_Humidity()[3] == True):
        Temp_Humidity_Values = Temp_Humidity()
        f.write("  Ambient Temperature = ")
        f.write(str(Temp_Humidity_Values[0])  )
        f.write("  ")
        f.write("Atmospheric Pressure = ")
        f.write(str(Temp_Humidity_Values[1]))
        f.write("  ")
        f.write("Ambient Humidity = ")
        f.write(str(Temp_Humidity_Values[2]))
        f.write("  ")
        f.write("\n")
        
    f.close()

def send(flag_WT,flag_TH,flag_ADC):
    
    if ((flag_WT == True) and (flag_TH == True) and (flag_ADC == True)):
        Temp_Humidity_Values = Temp_Humidity()
        Turbidity_Values = Turbidity()
        pH_Values = pH2_0()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())+"&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0])+"&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == True) and (flag_TH == True) and (flag_ADC == True)")
        
        
    elif ((flag_WT == True)  and (flag_TH == True)  and (flag_ADC == False)):
        Temp_Humidity_Values = Temp_Humidity()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())+"&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == True)  and (flag_TH == True)  and (flag_ADC == False)")
        
    elif ((flag_WT == True) and (flag_TH == False)  and (flag_ADC == True)):
        Turbidity_Values = Turbidity()
        pH_Values = pH2_0()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())+"&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == True) and (flag_TH == False)  and (flag_ADC == True")
        
    elif ((flag_WT == True) and (flag_TH == False)  and (flag_ADC == False)):
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature()))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == True) and (flag_TH == False)  and (flag_ADC == False)")
        
    elif ((flag_WT == False)  and (flag_TH == True)  and (flag_ADC == True)):
        Turbidity_Values = Turbidity()
        pH_Values = pH2_0()
        Temp_Humidity_Values = Temp_Humidity()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0])+"&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == False)  and (flag_TH == True)  and (flag_ADC == True)")
        
        
    elif ((flag_WT == False) and (flag_TH == True)  and (flag_ADC == False)):
        Temp_Humidity_Values = Temp_Humidity()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == False) and (flag_TH == True)  and (flag_ADC == False)")
       
        
    elif ((flag_WT == False)  and (flag_TH == False)  and (flag_ADC == True)):
        Turbidity_Values = Turbidity()
        pH_Values = pH2_0()
        enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0]))
        flag_TH = False
        flag_ADC = False
        flag_WT = False
        print("flag_WT == False)  and (flag_TH == False)  and (flag_ADC == True)")


def write_and_send3(flag_ADC, flag_TH, flag_WT):

    fecha_hora_formateada = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    f = open("datos.cvc","a")
    f.write(fecha_hora_formateada)

    try:
        if flag_WT == True:
            f.write("  WaterTemperature = ")
            f.write(str(WaterTemperature()))
            enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature()))
            flag_WT = False

        if flag_ADC == True:
            Turbidity_Values = Turbidity()
            pH_Values = pH2_0()
            f.write("  ")
            f.write("  Ph = ")
            f.write(str(pH2_0()))
            f.write("  ")
            f.write("  Turbidity = ")
            f.write(str(Turbidity()))
            f.write("  ")
            enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field2="+str(pH_Values[0])+"&field3="+str(Turbidity_Values[0]))
            flag_ADC = False

        if flag_TH == True:
            Temp_Humidity_Values = Temp_Humidity()
            f.write("  Ambient Temperature = ")
            f.write(str(Temp_Humidity_Values[0]))
            f.write("  ")
            f.write("Atmospheric Pressure = ")
            f.write(str(Temp_Humidity_Values[1]))
            f.write("  ")
            f.write("Ambient Humidity = ")
            f.write(str(Temp_Humidity_Values[2]))
            f.write("  ")
            enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field4="+str(Temp_Humidity_Values[0])+"&field5="+str(Temp_Humidity_Values[1])+"&field6="+str(Temp_Humidity_Values[2]))
            flag_TH = False

    except Exception as e:
        print(e)
        f.write("    Error:   \n")

    f.close()





    
def main():
    
    flag_WT = False
    flag_TH = False
    flag_ADC = False
   
    GPIO.setwarnings(False)
    
    toggle_gpio("on",16)     # gpio 16 3.3v // gpio 20 5.0vtoggle_gpio("on",20)
    toggle_gpio("on",20)
    time.sleep(1)
    
    flag_ADC = scan_devices(flag_TH,flag_ADC)[1]
    flag_TH = scan_devices(flag_TH,flag_ADC)[0]
    flag_WT = get_temperature_and_check_threshold(flag_WT)
    write2(flag_ADC,flag_TH,flag_WT)
    send(flag_WT,flag_TH,flag_ADC)
    
    toggle_gpio("off",16)
    toggle_gpio("off",20)
    
    
while(1):
    main()
    time.sleep(600)
