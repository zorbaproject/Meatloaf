#!/usr/bin/python3

#sudo pip3 install Adafruit-BMP

import Adafruit_BMP.BMP085 as BMP085 # Imports the BMP library

# Create an 'object' containing the BMP180 data
bmp_sensor = BMP085.BMP085(busnum=5)
print('Temp = '+str(bmp_sensor.read_temperature())) # Temperature in Celcius
print('Pressure = '+str(bmp_sensor.read_pressure())) # The local pressure
print('Altitude = '+str(bmp_sensor.read_altitude())) # The current altitude
print('Sealevel Pressure = '+str(bmp_sensor.read_sealevel_pressure())) # The sea-level pressure

#Mean Sea Level = msl, pressione attuale al livello del mare, da bollettino meteo

msl = 1015.00
#https://www.osmer.fvg.it/stazioni.php?ln=&m=0

#Posso calcolare la msl se ho un punto con pressione e altitudine nota:
#Es:
hpa_pressure = 994.24
known_alt = 194
msl = (hpa_pressure /(pow((1-(known_alt/44330.0)),5.255)))

temperature = bmp_sensor.read_temperature()
pressure = bmp_sensor.read_pressure()
hpa_pressure = pressure/100

altitude_adjusted = (44330.0*(1-pow(hpa_pressure/msl,1/5.255)))


print("hPa pressure: "+str(hpa_pressure))
print("Altitude adjusted: "+str(altitude_adjusted))
