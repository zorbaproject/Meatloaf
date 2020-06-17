#!/usr/bin/python3
# -*- coding: utf-8 -*-

from time import sleep
import math
import re

import serial  #https://pyserial.readthedocs.io/en/latest/shortintro.html

rangefinderTTY = '/dev/ttyUSB0'
baudrate = 19200
ledoncommand = b'O'
distancecommand = b'D'
ledoncode = "OK!"  #This is the response to look for after turning on led
distancecode = "m," #This is the response to look for after requesting a distance measurement

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
        except:
            sleep(0.5)
        sleep(0.5)


rangefinderTTY = searchRangefinder(['/dev/ttyUSB0','/dev/ttyUSB1'])
if rangefinderTTY != None:
    print("Found rangefinder on "+rangefinderTTY)
    getMeasure()
else:
    print("Didn't find a rangefinder on any suggested serial port.")
