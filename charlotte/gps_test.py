#!/usr/bin/python3
import serial
import time
import string
#import pynmea2

#sudo pip3 install pynmea2   #Not really necessary
#sudo pip3 install pyserial

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

relayPin = 25
GPIO.setup(relayPin, GPIO.OUT)



#Add into /boot/config.txt
#dtparam=spi=on
#dtoverlay=pi3-disable-bt
#core_freq=250
#enable_uart=1
#force_turbo=1
#init_uart_baud=9600

#replace /boot/cmdline.txt with
#dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles

#http://aprs.gids.nl/nmea/
#https://github.com/Knio/pynmea2

#Connect:
#VCC to RPi 5V
#GND to RPi GND
#RX to RPi GPIO 14
#TX to RPi GPIO 15

#Relay:
#GPIO25



GPIO.output(relayPin, True)



while True:
 port="/dev/ttyAMA0"
 ser=serial.Serial(port, baudrate=9600, timeout=0.5)
 #reader = pynmea2.NMEAStreamReader()
 try:
   newdata=ser.readline().decode(encoding="ascii")
 except:
   newdata=str(ser.readline())
 newdata=newdata.replace("\\r", "").replace("\\n", "")
 newdata=newdata.replace("\r", "").replace("\n", "")
 newdata=newdata.replace("'","")
 if newdata.startswith("b"):
     newdata=newdata[1:]

 if "," not in newdata:
     continue

 #print(newdata)

 if newdata.find("GSA,") > 1:
    print("Fix type (1: no, 2: 2D, 3: 3D): "+str(newdata.split(',')[2]))


 if newdata.find("ZDA,") > 1:
   #Not supported by gy-gps6mv2
   print("ZDA:" + newdata)

 if newdata.find("GGA,") > 1:
   lat=str(float(newdata.split(",")[2][0:2])+(float(newdata.split(",")[2][2:])/60)) +newdata.split(",")[3]
   lon=str(float(newdata.split(",")[4][0:3])+(float(newdata.split(",")[4][3:])/60)) + newdata.split(",")[5]
   fix=newdata.split(",")[6]
   sat=newdata.split(",")[7]
   alt=newdata.split(",")[9]
   print("Number of satellites: "+str(sat))
   print("Fix quality 0 = Invalid, 1 = GPS, 2 = Differential GPS: "+str(fix))
   print("Lat,long,alt:",lat,lon,alt)

 if newdata.find("RMC") > 1:
   #newmsg=pynmea2.parse(newdata)
   #lat=newmsg.latitude
   #lng=newmsg.longitude
   #gps = "Latitude=" + str(lat) + str(newmsg.lat_dir) + " and Longitude=" + str(lng)+ str(newmsg.lon_dir)
   #print(gps)
   #print(newmsg.timestamp)
   date = newdata.split(",")[9]
   time = newdata.split(",")[1]
   gpsdatetime = date[0:2] + "/" + date[2:4] + "/" +date[4:6] + " " + time[0:2] + ":" + time[2:4] + ":" +time[4:6] + " "
   print("GPS Time: "+gpsdatetime)



