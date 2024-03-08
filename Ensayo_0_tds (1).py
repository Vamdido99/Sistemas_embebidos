import Adafruit_ADS1x15
import time

# Define the function to read the TDS sensor value.
def read_tds_value():
    #nonlocal average_voltage, tds_value, static_time
        
    # Define the ADS115 ADC (16-bit) instance.
    adc = Adafruit_ADS1x15.ADS1115()

    # Define the TDS pin (A1 in the original code) a channel in the ADC.
    TDS_SENSOR_PIN = 0

    # Define the VREF voltage (5.0V in the original code).
    VREF = 5.0
    
    # define max value on bits for adc when he read 5.0v
    max_val1 = 25845

    # Define the number of samples (30 in the original code).
    SCOUNT = 30

    # Define the variables for storing the average voltage and TDS value.
    average_voltage = 0.0
    tds_value = 0.0

    # Define the temperature (25°C in the original code).
    temperature = 25

    # Define the variable for storing the sum of the samples.
    sum_values = 0.0
    
    buff = []

    # Take SCOUNT samples.
    for i in range(SCOUNT):
        # Read the value from the TDS sensor channel.
        value = adc.read_adc(TDS_SENSOR_PIN, gain=2/3)
        buff.append(value)
        print(i, value)
        time.sleep(0.4)
    
    buff.sort()
    avg = sum(buff[12:19]) / len(buff[12:19])
    x1=int(avg)
    
 
    average_voltage = x1 / max_val1  * VREF
    
    print("average_voltage",average_voltage)
    
    # Calculate the TDS value.
    compensation_coefficient = 1.0 + 0.02 * (temperature - 25.0)
    compensation_voltage = average_voltage / compensation_coefficient
    tds_value = (133.42 * compensation_voltage * compensation_voltage * compensation_voltage - 255.86 * compensation_voltage * compensation_voltage + 857.39 * compensation_voltage) * 0.5

    # Set the initial static time.
    static_time = time.time()
    print("process 0")
    
    return tds_value

def volt_chanel(chanel):
    max_val1= 25845
    voltage_ref1= 5
    adc = Adafruit_ADS1x15.ADS1115()
    value= adc.read_adc(chanel, gain=2/3) # value= adc.read_adc(chanel, gain=2/3)
    v1 = value / max_val1 * voltage_ref1
    return value,v1

# Read the TDS value repeatedly.

#read_tds_value()
print(volt_chanel(0))
 #avg = np.average(buf)
  #      t=(round(avg,1))
print("TDS Value:  ppm",round(read_tds_value(),1))
print("process 2")
        

# Call the function to start reading the TDS value.
#read_tds_value()