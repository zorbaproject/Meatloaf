#!/usr/bin/python3
# -*- coding: utf-8 -*-

from time import sleep
import math
import re
import os

import serial  #https://pyserial.readthedocs.io/en/latest/shortintro.html

rangefinderTTY = '/dev/ttyUSB0'
baudrate = 19200
ledoncommand = b'O'
distancecommand = b'D'
ledoncode = "OK!"  #This is the response to look for after turning on led
distancecode = "m," #This is the response to look for after requesting a distance measurement
rangefinderAddress = ""

def findUSBaddress(myconverter, mydriver = ""):
    addr = ""
    if mydriver == "":
        mydriver = myconverter
    os.system("dmesg > /tmp/dmesg.log")
    text_file = open("/tmp/dmesg.log", "r")
    mylines = text_file.read().split("\n")
    text_file.close()
    for myline in mylines:
        if ': '+myconverter+' converter detected' in myline:
            addr = re.sub('\[.*\] '+mydriver+' (.*):.*','\g<1>', myline)
    return addr

def reconnectUSB(myaddress, mydriver):
    print("Trying to reconnect USB device "+myaddress)
    os.system("sudo sh -c 'echo -n \""+myaddress+"\" > /sys/bus/usb/drivers/"+mydriver+"/unbind'")
    sleep(1)
    #sudo sh -c 'ls -hal /root/ > /root/test.out'
    os.system("sudo sh -c 'echo -n \""+myaddress+"\" > /sys/bus/usb/drivers/"+mydriver+"/bind'")
    sleep(1)

def searchRangefinder(ttys = ['/dev/ttyUSB0']):
    global rangefinderTTY
    global baudrate
    global ledoncommand
    global distancecommand
    global ledoncode
    global distancecode
    line = ""
    for mytty in ttys:
        try:
            with serial.Serial(mytty, baudrate, timeout=1) as ser:
                ser.write(ledoncommand)
                line = ser.readline().decode('ascii')   # read a '\n' terminated line
                if ledoncode in line:
                    return mytty
        except:
            sleep(0.5)
        sleep(0.5)
    return None

def getMeasure():
    global rangefinderTTY
    global baudrate
    global ledoncommand
    global distancecommand
    global ledoncode
    global distancecode
    global rangefinderAddress
    line = ""
    dist = 0.0
    while True:
        try:
            with serial.Serial(rangefinderTTY, baudrate, timeout=1) as ser:
                if ledoncode in line:
                    ser.write(distancecommand)
                else:
                    ser.write(ledoncommand)
                line = ser.readline().decode('ascii')   # read a '\n' terminated line
                #print(line)
                if distancecode in line:
                    dist = float(re.sub("[^0-9]([0-9\.]*)","\g<1>",line.split(distancecode)[0]))
                    print("Distance: "+str(dist))
        except serial.serialutil.SerialException as e:
            sleep(0.5)
            reconnectUSB(rangefinderAddress, "ch341")
            rangefinderTTY = searchRangefinder(['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyACM0','/dev/ttyACM1'])
        except UnicodeDecodeError as e:
            print("Unable to decode string, retrying")
            sleep(0.5)
        #else:
        #    sleep(0.5)
        sleep(0.5)


rangefinderTTY = searchRangefinder(['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyACM0','/dev/ttyACM1'])
# dmesg | grep ': ch341-uart converter detected' |sed 's/\[.*\] ch341 \(.*\):.*/\1/g' | tail -n1
rangefinderAddress = findUSBaddress("ch341-uart", "ch341")
print(rangefinderAddress)
if rangefinderTTY != None:
    print("Found rangefinder on "+rangefinderTTY)
    getMeasure()
else:
    print("Didn't find a rangefinder on any suggested serial port.")
