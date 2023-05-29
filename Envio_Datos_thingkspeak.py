import requests
import serial
import time

# Configuración del puerto serial
ser = serial.Serial('/dev/serial0', 9600, timeout=5)  # Ajusta el puerto serial y la velocidad según tu configuración
a = 0
# Configuración del canal y la clave de escritura
channel_id = '2136499'  # Reemplaza con tu ID de canal
write_key = '3WLMTG6CBXPKOBK5'  # Reemplaza con tu clave de escritura

# URL de la API de ThingSpeak
url = f'https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5'
#enviar=requests.get("https://api.thingspeak.com/update?api_key=3WLMTG6CBXPKOBK5&field1="+str(WaterTemperature())
def send_to_thingspeak(data):
    # Parámetros de datos a enviar al servidor
    payload = {'field1': data}  # Ajusta 'field1' según los campos definidos en tu canal

    # Envía la solicitud HTTP POST a ThingSpeak
    response = requests.post(url, data=payload)

    # Verifica el código de estado de la respuesta
    if response.status_code == 200:
        print('Dato enviado correctamente a ThingSpeak.')
    else:
        print('Error al enviar el dato a ThingSpeak.')

# Función para enviar un comando AT al módulo SIM800L
def send_at_command(command):
    ser.write((command + '\r\n').encode())
    time.sleep(1)
    response = ser.readlines()
    for line in response:
        print(line.decode().strip())
while True:
    a=a+1
# Configura el módulo SIM800L
    send_at_command('AT')  # Comando AT para verificar la comunicación con el módulo SIM800L
    send_at_command('AT+SAPBR=3,1,"APN","orangeworld"')  # Reemplaza "TU_APN" con el APN de tu operador móvil
    send_at_command('AT+SAPBR=1,1')  # Activa el contexto GPRS
    send_at_command('AT+CIICR')  # Establece la conexión GPRS
    send_at_command('AT+CIFSR')  # Obtiene la dirección IP asignada
    send_at_command('AT+HTTPINIT')  # Inicializa la funcionalidad HTTP
    send_at_command('AT+HTTPPARA="CID",1')  # Establece el contexto HTTP

# Ejemplo de uso
    data_to_send = a  # Reemplaza con el dato que deseas enviar a ThingSpeak (como una cadena)

    send_to_thingspeak(data_to_send)

# Cierra la conexión GPRS y el puerto serial
    send_at_command('AT+HTTPTERM')  # Termina la funcionalidad HTTP
    send_at_command('AT+SAPBR=0,1')  # Desactiva el contexto GPRS
  #  ser.close()
    time.sleep(20)