#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, os
import time
from time import sleep
import json
import math
import re
from datetime import datetime
import numpy as np

import shutil
import platform

import ezdxf

isWindows = False
arch = platform.architecture()[0]
if platform.system() == "Windows":
    isWindows = True

try:
    import serial
    import PyLidar3
    #Devono essere abilitati i 1-Wire
    os.system('sudo modprobe w1-gpio')
    os.system('sudo modprobe w1-therm')
    import RPi.GPIO as GPIO
    import w1thermsensor
    import board
    import busio
    #import adafruit_lsm303_accel
    #import adafruit_lsm303dlh_mag
    import Adafruit_BMP.BMP085 as BMP085
    import smbus
    isRPI = True
    print("Running on RPi")
except:
    isRPI = False
    print("Not running on RPi")



from PySide2.QtWidgets import QApplication
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCore import QDir
from PySide2.QtCore import Qt
from PySide2.QtCore import QTimer
from PySide2.QtCore import Signal
from PySide2.QtCore import QSize
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QThread
from PySide2.QtWidgets import QTableWidget
from PySide2.QtWidgets import QTableWidgetItem
from PySide2.QtWidgets import QTableWidgetSelectionRange
from PySide2.QtWidgets import QGraphicsScene
from PySide2.QtWidgets import QDialog
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QDialogButtonBox
from PySide2.QtSvg import QSvgGenerator
from PySide2.QtGui import QPainter
from PySide2.QtGui import QPainterPath
from PySide2.QtGui import QPolygonF
from PySide2.QtGui import QPen
from PySide2.QtGui import QColor
from PySide2.QtGui import QTransform
from PySide2.QtGui import QFont
from PySide2.QtCore import QPointF

import threading

print("All modules imported")

toSleep = 1
samples = 5 #number of samples to take for calculating average

class hmc5883l:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0), mycalibr=None):
        self.hmcbus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180
        
        self.CompassCalibration = mycalibr

        (reg, self.__scale) = self.__scales[gauss]
        self.hmcbus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.hmcbus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.hmcbus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.hmcbus.read_i2c_block_data(self.address, 0x00)
        #print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        
        try:
          if self.CompassCalibration != None:
              #https://pololu.github.io/zumo-shield-arduino-library/_l_s_m303_8h_source.html
              x = x - (self.CompassCalibration["MagMinX"] + self.CompassCalibration["MagMaxX"]) / 2
              y = y - (self.CompassCalibration["MagMinY"] + self.CompassCalibration["MagMaxY"]) / 2
              z = z - (self.CompassCalibration["MagMinZ"] + self.CompassCalibration["MagMaxZ"]) / 2
        except:
          pass
        
        headingRad = math.atan2(y, x)
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "Declination: " + self.degrees(self.declination()) + "\n" \
               "Heading: " + self.degrees(self.heading()) + "\n"


class getGPS(QThread):
    gpsposition = Signal(list)
    gpstime = Signal(int)
    def __init__(self, parent, mydata = ""):
        QThread.__init__(self)
        self.myparent = parent
        self.setTerminationEnabled(True)
        if not isRPI:
            self.exit()
        self.relayPinGPS = 25
        self.GPSport="/dev/ttyAMA0"

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(relayPinGPS, GPIO.OUT)
        self.turnOFF()

    def turnON(self):
        GPIO.output(relayPin, True)

    def turnOFF(self):
        GPIO.output(relayPin, False)

    def getTime(self):
        self.turnON()
        active = True
        while active:
           ser=serial.Serial(self.GPSport, baudrate=9600, timeout=0.5)
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

           if newdata.find("GSA,") > 1:
               print("Fix type (1: no, 2: 2D, 3: 3D): "+str(newdata.split(',')[2]))
               pass

           if newdata.find("RMC") > 1:
               date = newdata.split(",")[9]
               time = newdata.split(",")[1]
               gpsdatetime = date[0:2] + "/" + date[2:4] + "/" +date[4:6] + " " + time[0:2] + ":" + time[2:4] + ":" +time[4:6] + " "
               print("GPS Time: "+gpsdatetime)
               gpstime = 0
               self.position.emit(gpstime)
               active = False
        self.turnOFF()


    def getPosition(self):
        self.turnON()
        active = True
        while active:
           ser=serial.Serial(self.GPSport, baudrate=9600, timeout=0.5)
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

           if newdata.find("GSA,") > 1:
               print("Fix type (1: no, 2: 2D, 3: 3D): "+str(newdata.split(',')[2]))
               pass

           if newdata.find("GGA,") > 1:
               lat=str(float(newdata.split(",")[2][0:2])+(float(newdata.split(",")[2][2:])/60)) +newdata.split(",")[3]
               lon=str(float(newdata.split(",")[4][0:3])+(float(newdata.split(",")[4][3:])/60)) + newdata.split(",")[5]
               fix=newdata.split(",")[6]
               sat=newdata.split(",")[7]
               alt=newdata.split(",")[9]
               print("Number of satellites: "+str(sat))
               print("Fix quality 0 = Invalid, 1 = GPS, 2 = Differential GPS: "+str(fix))
               print("Lat,long,alt:",lat,lon,alt)
               myfix = [lat,lon,alt,sat,fix]
               self.gpsposition.emit(myfix)
               self.myparent.w.statusbar.showMessage(str(myfix))
               active = False
        self.turnOFF()



class getData(QThread):
    GotScan = Signal(list)
    CompassCal = Signal(dict)
    #######################################Here we read temperature, pressure, distance, and 3-axis position
    def __init__(self, parent, mydata = ""):
        QThread.__init__(self)
        self.myparent = parent
        self.setTerminationEnabled(True)
        if not isRPI:
            self.exit()
        #Data for the rangefinder
        self.rangefinderTTY = '/dev/ttyUSB0'
        plausibleTTYs = ['/dev/ttyUSB0','/dev/ttyUSB1']
        self.RFbaudrate = 19200
        self.ledoffcommand = b'C'
        self.ledoncommand = b'O'
        self.distancecommand = b'D'
        self.ledoncode = "OK!"  #This is the response to look for after turning on led
        self.distancecode = "m," #This is the response to look for after requesting a distance measurement
        try:
            self.declination = (self.myparent.mycfg['declination'][0],self.myparent.mycfg['declination'][1])
        except:
            self.declination = (4,3)    #Degrees, minutes (https://www.magnetic-declination.com/#)
        print("Declination: " + str(self.declination))
        #Cerco i sensori
        try:
            self.lsmbus = self.getLSM303_bus(1)
        except:
            print("Unable to find LSM303DLH compass and inclinometer.")
            self.lsmbus = None
        try:
            self.compass3 = hmc5883l(port = 3, gauss = 4.7, declination = self.declination, mycalibr = self.myparent.mycfg["CompassCalibration"]["3"])
        except:
            print("Unable to find HCM5883L compass and inclinometer.")
            self.compass3 = None
        self.rangefinderTTY = self.searchRangefinder(plausibleTTYs)
        print("Rangefinder: " + str(self.rangefinderTTY))
        try:
            self.tempsensor = w1thermsensor.W1ThermSensor()
        except:
            self.tempsensor = None
        #LiDAR
        self.scantime = 2
        if self.rangefinderTTY != None:
            plausibleTTYs.remove(self.rangefinderTTY)
        self.lidarport = self.findYDLidarX4(plausibleTTYs)
        #self.lidarport = "/dev/ttyUSB1"
        if self.lidarport == None and len(plausibleTTYs)>0:
            self.lidarport = plausibleTTYs[-1]
        print("Found Lidar on " + str(self.lidarport))
        self.lidarAddress = self.findUSBaddress("cp210x")
        # "dmesg | grep ': cp210x converter detected' |sed 's/\[.*\] cp210x \(.*\):.*/\1/g' | tail -n1"
        print("Lidar Address: "+self.lidarAddress)
        self.msl = 1013.0
        


    def __del__(self):
        print("Shutting down thread")

    def findUSBaddress(self, myconverter, mydriver = ""):
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

    def reconnectUSB(self, myaddress, mydriver):
        print("Trying to reconnect USB device "+myaddress)
        os.system("sudo sh -c 'echo -n \""+myaddress+"\" > /sys/bus/usb/drivers/"+mydriver+"/unbind'")
        sleep(1)
        #sudo sh -c 'ls -hal /root/ > /root/test.out'
        os.system("sudo sh -c 'echo -n \""+myaddress+"\" > /sys/bus/usb/drivers/"+mydriver+"/bind'")
        sleep(1)

    def run(self):
        global toSleep
        while True:
            #Compass calibration mode
            if self.myparent.w.calibraBussola.isChecked():
                headCalSurveys = 200
                while True:
                    if self.myparent.w.manualMode.isChecked():
                        break
                    time.sleep(0.1)
                try:
                    #self.myparent.compassdialoglbl.setText("Start moving compass...")
                    try:
                        MagMinX = int(self.myparent.CompassCalibration["1"]["MagMinX"])
                    except:
                        MagMinX = 1000000
                    try:
                        MagMaxX = int(self.myparent.CompassCalibration["1"]["MagMaxX"])
                    except:
                        MagMaxX = -1000000
                    try:
                        MagMinY = int(self.myparent.CompassCalibration["1"]["MagMinY"])
                    except:
                        MagMinY = 1000000
                    try:
                        MagMaxY = int(self.myparent.CompassCalibration["1"]["MagMaxY"])
                    except:
                        MagMaxY = -1000000
                    try:
                        MagMinZ = int(self.myparent.CompassCalibration["1"]["MagMinZ"])
                    except:
                        MagMinZ = 1000000
                    try:
                        MagMaxZ = int(self.myparent.CompassCalibration["1"]["MagMaxZ"])
                    except:
                        MagMaxZ = -1000000
                    try:
                        Mag3MinX = int(self.myparent.CompassCalibration["1"]["Mag3MinX"])
                    except:
                        Mag3MinX = 1000000
                    try:
                        Mag3MaxX = int(self.myparent.CompassCalibration["1"]["Mag3MaxX"])
                    except:
                        Mag3MaxX = -1000000
                    try:
                        Mag3MinY = int(self.myparent.CompassCalibration["1"]["Mag3MinY"])
                    except:
                        Mag3MinY = 1000000
                    try:
                        Mag3MaxY = int(self.myparent.CompassCalibration["1"]["Mag3MaxY"])
                    except:
                        Mag3MaxY = -1000000
                    try:
                        Mag3MinZ = int(self.myparent.CompassCalibration["1"]["Mag3MinZ"])
                    except:
                        Mag3MinZ = 1000000
                    try:
                        Mag3MaxZ = int(self.myparent.CompassCalibration["1"]["Mag3MaxZ"])
                    except:
                        Mag3MaxZ = -1000000
                    #https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration?view=all
                    #https://github.com/adafruit/Adafruit_LSM303DLH_Mag/blob/master/examples/calibration/calibration.ino
                    #https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
                    #https://www.instructables.com/Configure-read-data-calibrate-the-HMC5883L-digital/
                    for i in range(headCalSurveys):
                        xMag,yMag,zMag = self.getLSM303_heading(self.lsmbus)
                        if xMag < MagMinX: MagMinX = xMag
                        if xMag > MagMaxX: MagMaxX = xMag
                        if yMag < MagMinY: MagMinY = yMag
                        if yMag > MagMaxY: MagMaxY = yMag
                        if zMag < MagMinZ: MagMinZ = zMag
                        if zMag > MagMaxZ: MagMaxZ = zMag
                        #heading1 = self.get_heading(xMag,yMag,zMag, self.declination)
                        xMag3,yMag3,zMag3 = self.compass3.axes()
                        if xMag3 == None: xMag3 = 0
                        if yMag3 == None: yMag3 = 0
                        if zMag3 == None: zMag3 = 0
                        if xMag3 < Mag3MinX: Mag3MinX = xMag3
                        if xMag3 > Mag3MaxX: Mag3MaxX = xMag3
                        if yMag3 < Mag3MinY: Mag3MinY = yMag3
                        if yMag3 > Mag3MaxY: Mag3MaxY = yMag3
                        if zMag3 < Mag3MinZ: Mag3MinZ = zMag3
                        if zMag3 > Mag3MaxZ: Mag3MaxZ = zMag3
                        #heading3 = self.compass3.heading()                        
                        self.myparent.compassdialoglbl.setText("Please move compass all around for "+str(int((headCalSurveys-i)*0.1))+" seconds \n"+str([xMag,yMag,zMag])+" "+str([xMag3,yMag3,zMag3]))
                        time.sleep(0.1)
                    mycompasscal = {"1":{"MagMinX":MagMinX,"MagMaxX":MagMaxX,"MagMinY":MagMinY,"MagMaxY":MagMaxY,"MagMinZ":MagMinZ,"MagMaxZ":MagMaxZ}, 
                                    "3":{"MagMinX":int(Mag3MinX),"MagMaxX":int(Mag3MaxX),"MagMinY":int(Mag3MinY),"MagMaxY":int(Mag3MaxY),"MagMinZ":int(Mag3MinZ),"MagMaxZ":int(Mag3MaxZ)}}
                    self.myparent.compassdialoglbl.setText("Done")
                    self.CompassCal.emit(mycompasscal)
                except:
                    self.myparent.compassdialoglbl.setText("Error calibrating compass")
                    pass
                continue
            #Manual mode
            if self.myparent.w.manualMode.isChecked():
                self.stopRangefinder()
                sleep(toSleep)
                continue
            #Missing lidar mode
            if self.lidarport != None:
                myscan = self.scanYDLidarX4()
                #self.myparent.LidarScanDone(myscan)
                self.GotScan.emit(myscan)
            #Normal mode
            self.requiredData = {}
            try:
                self.requiredData["temperature"] = self.readTemp()
            except:
                self.requiredData["temperature"] = -127
            try:
                temperature, pressure, alt_adjusted = self.readBMP()
                self.requiredData["temperature"] = temperature
                self.requiredData["pressure"] = pressure
                self.requiredData["bar_altitude"] = alt_adjusted
            except:
                self.requiredData["temperature"] = 0
                self.requiredData["pressure"] = 0
                self.requiredData["bar_altitude"] = -10000
            try:
                xAccl,yAccl,zAccl = self.getLSM303_accel(self.lsmbus)
                #xMag,yMag,zMag = self.getLSM303_heading(self.lsmbus)
                self.requiredData["sideTilt"] = self.get_x_rotation(xAccl,yAccl,zAccl)
                self.requiredData["frontalInclination"] = self.get_y_rotation(xAccl,yAccl,zAccl)
            except:
                self.requiredData["sideTilt"] = 0.0
                self.requiredData["frontalInclination"] = 0.0
            try:
                #declination = (0,0)
                headSurveys = 10
                heading1 = 0
                heading3 = 0
                for i in range(headSurveys):
                    xMag,yMag,zMag = self.getLSM303_heading(self.lsmbus)
                    heading1 = heading1 + (self.get_heading(xMag,yMag,zMag, self.declination)/headSurveys)
                    heading3 = heading3 + (self.compass3.heading()/headSurveys)
                    time.sleep(0.1)
                print("heading 0° incl: " + str(heading1))
                print("heading 90° incl: " + str(heading3))
                if bool(self.requiredData["frontalInclination"] > -45 and self.requiredData["frontalInclination"] < 45) or bool(self.requiredData["frontalInclination"] > 135 and self.requiredData["frontalInclination"] < -135):
                    self.requiredData["heading"] = heading1
                    print("0° choosen")
                    self.myparent.w.statusbar.showMessage("Compass on bus 1 choosen: "+str(int(heading1))+ "; bus 3: "+str(int(heading3)))
                else:
                    self.requiredData["heading"] = heading3
                    print("90° choosen")
                    self.myparent.w.statusbar.showMessage("Compass on bus 1: "+str(int(heading1))+ "; bus 3 choosen: "+str(int(heading3)))
            except:
                self.requiredData["heading"] = 0.0
            try:
                self.requiredData["distance"] = self.getDistance()
            except:
                self.requiredData["distance"] = 0.0
            self.myparent.setRequiredData(self.requiredData)
            sleep(toSleep)
        return

    def findYDLidarX4(self, ttys = ["/dev/ttyUSB0", "/dev/ttyUSB1"]):
        print("Searching for Lidar on: ")
        try:
            for tmptty in ttys:
                print(tmptty)
                Obj = PyLidar3.YdLidarX4(tmptty) #PyLidar3.your_version_of_lidar(port,chunk_size)
                if(Obj.Connect()):
                    print(Obj.GetDeviceInfo())
                    Obj.Disconnect()
                    return tmptty
            t = 0/0
        except:
            return None

    def scanYDLidarX4(self):
        myscan = [0.0 for deg in range(360)] #we have one value for every angle
        if self.myparent.w.nolidar.isChecked():
            return myscan
        Obj = PyLidar3.YdLidarX4(self.lidarport)
        if(Obj.Connect()):
            gen = Obj.StartScanning()
            t = time.time() # start time
            scanlist = []
            while (time.time() - t) < self.scantime:
                scanlist.append(next(gen))
                time.sleep(0.5)
            Obj.StopScanning()
            Obj.Disconnect()
            for i in range(360):
                sum = 0.0
                for tmpscan in scanlist:
                    sum = sum +tmpscan[i]
                myscan[i] = (float(sum)/len(scanlist))/1000.0
            #print(len(scanlist))
        else:
            print("Error connecting to device")
        return myscan
    
    def readTemp(self):
        #Leggo la temperatura
        temperature_in_celsius = self.tempsensor.get_temperature()
        return temperature_in_celsius

    def readBMP(self):
        bmp_sensor = BMP085.BMP085(busnum=5)
        #Mean Sea Level = msl, pressione attuale al livello del mare, da bollettino meteo
        #https://www.osmer.fvg.it/stazioni.php?ln=&m=0
        msl = self.msl

        temperature = bmp_sensor.read_temperature()
        pressure = bmp_sensor.read_pressure()
        hpa_pressure = pressure/100

        altitude_adjusted = (44330.0*(1-pow(hpa_pressure/msl,1/5.255)))
        return temperature, hpa_pressure, altitude_adjusted


    def calibrateBMP(self, known_alt = 0.0):
        t, hpa_pressure, alt = self.readBMP()
        msl = (hpa_pressure /(pow((1-(known_alt/44330.0)),5.255)))
        self.msl = msl
        #Add to configure
        return msl

    def stopRangefinder(self):
        line = ""
        try:
            with serial.Serial(self.rangefinderTTY, self.RFbaudrate, timeout=1) as ser:
                ser.write(self.ledoffcommand)
                line = ser.readline().decode('ascii')   # read a '\n' terminated line
                #print(line)
        except:
            sleep(0.5)
        #sleep(0.5)
        return None

    def searchRangefinder(self, ttys = ['/dev/ttyUSB0']):
        line = ""
        for mytty in ttys:
            try:
                with serial.Serial(mytty, self.RFbaudrate, timeout=1) as ser:
                    ser.write(self.ledoncommand)
                    line = ser.readline().decode('ascii')   # read a '\n' terminated line
                    if self.ledoncode in line:
                        return mytty
            except:
                sleep(0.5)
            sleep(0.5)
        return None

    def getDistance(self):
        line = ""
        dist = 0.0
        lookfordistance = True
        errcount = 0
        while lookfordistance:
            try:
                with serial.Serial(self.rangefinderTTY, self.RFbaudrate, timeout=1) as ser:
                    if self.ledoncode in line:
                        ser.write(self.distancecommand)
                    else:
                        ser.write(self.ledoncommand)
                    line = ser.readline().decode('ascii')   # read a '\n' terminated line
                    #print(line)
                    if self.distancecode in line:
                        dist = float(re.sub("[^0-9]([0-9\.]*)","\g<1>",line.split(self.distancecode)[0]))
                        #print("Distance: "+str(dist))
                        #Probably better to just keep laser on
                        ser.write(self.ledoncommand)
                        lookfordistance = False
            except:
                errcount = errcount + 1
                sleep(0.1)
                if errcount > 10:
                    print("Unable to read valid data from rangefinder")
                    break
        try:
            t = float(dist)
        except:
            dist = 0.0
            print("Error: Distance value must be a number")
        return dist

    def dist(self, a,b):
        return math.sqrt((a*a)+(b*b))

    def get_y_rotation(self, x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return math.degrees(radians)

    def get_x_rotation(self, x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)


    def get_headingOLD(self, x,y,z):
        if math.fabs(z) < 3:
            print("Tilting too much, heading measurement could be unaccurate")
        radians = math.atan2(y,x)
        heading = math.degrees(radians)
        if heading <0:
            heading = heading + 360
        return heading

    def get_heading(self, x,y,z, declination):
      tmpX = x
      tmpY = y
      tmpZ = z
      try:
          if self.myparent.CompassCalibration != None:
              #https://pololu.github.io/zumo-shield-arduino-library/_l_s_m303_8h_source.html
              #self.myparent.w.statusbar.showMessage("Calibration: "+str(self.myparent.CompassCalibration["1"]))
              tmpX = tmpX - (self.myparent.CompassCalibration["1"]["MagMinX"] + self.myparent.CompassCalibration["1"]["MagMaxX"]) / 2
              tmpY = tmpY - (self.myparent.CompassCalibration["1"]["MagMinY"] + self.myparent.CompassCalibration["1"]["MagMaxY"]) / 2
              tmpZ = tmpZ - (self.myparent.CompassCalibration["1"]["MagMinZ"] + self.myparent.CompassCalibration["1"]["MagMaxZ"]) / 2
      except:
          pass
      headingRad = math.atan2(tmpY, tmpX)
      (degrees, minutes) = declination
      declDegrees = degrees
      declMinutes = minutes
      declinationD = (degrees + minutes / 60) * math.pi / 180
      headingRad += declinationD

      # Correct for reversed heading
      if headingRad < 0:
        headingRad += 2 * math.pi

      # Check for wrap and compensate
      elif headingRad > 2 * math.pi:
        headingRad -= 2 * math.pi

      # Convert to degrees from radians
      headingDeg = headingRad * 180 / math.pi
      return headingDeg

    #
    def getLSM303_bus(self, busnum = 1):
    # Get I2C bus
      bus = smbus.SMBus(busnum)

    # LSM303DLHC Accl address, 0x19(25)
    # Select control register1, 0x20(32)
    #		0x27(39)	Acceleration data rate = 10Hz, Power ON, X, Y, Z axis enabled
      bus.write_byte_data(0x19, 0x20, 0x27)
    # LSM303DLHC Accl address, 0x19(25)
    # Select control register4, 0x23(35)
    #		0x00(00)	Continuos update, Full scale selection = +/-2g,
      bus.write_byte_data(0x19, 0x23, 0x00)

      time.sleep(0.5)
      return bus


    def getLSM303_accel(self, bus):
    # LSM303DLHC Accl address, 0x19(25)
    # Read data back from 0x28(40), 2 bytes
    # X-Axis Accl LSB, X-Axis Accl MSB
      data0 = bus.read_byte_data(0x19, 0x28)
      data1 = bus.read_byte_data(0x19, 0x29)

    # Convert the data
      xAccl = data1 * 256 + data0
      if xAccl > 32767 :
            xAccl -= 65536

    # LSM303DLHC Accl address, 0x19(25)
    # Read data back from 0x2A(42), 2 bytes
    # Y-Axis Accl LSB, Y-Axis Accl MSB
      data0 = bus.read_byte_data(0x19, 0x2A)
      data1 = bus.read_byte_data(0x19, 0x2B)

    # Convert the data
      yAccl = data1 * 256 + data0
      if yAccl > 32767 :
            yAccl -= 65536

    # LSM303DLHC Accl address, 0x19(25)
    # Read data back from 0x2C(44), 2 bytes
    # Z-Axis Accl LSB, Z-Axis Accl MSB
      data0 = bus.read_byte_data(0x19, 0x2C)
      data1 = bus.read_byte_data(0x19, 0x2D)

    # Convert the data
      zAccl = data1 * 256 + data0
      if zAccl > 32767 :
            zAccl -= 65536

      return (xAccl,yAccl,zAccl)

    def getLSM303_heading(self, bus):
    # LSM303DLHC Mag address, 0x1E(30)
    # Select MR register, 0x02(02)
    #		0x00(00)	Continous conversion mode
      bus.write_byte_data(0x1E, 0x02, 0x00)
    # LSM303DLHC Mag address, 0x1E(30)
    # Select CRA register, 0x00(00)
    #		0x10(16)	Temperatuer disabled, Data output rate = 15Hz
      bus.write_byte_data(0x1E, 0x00, 0x10)
    # LSM303DLHC Mag address, 0x1E(30)
    # Select CRB register, 0x01(01)
    #		0x20(32)	Gain setting = +/- 1.3g
      bus.write_byte_data(0x1E, 0x01, 0x20)

      time.sleep(0.5)

    # LSM303DLHC Mag address, 0x1E(30)
    # Read data back from 0x03(03), 2 bytes
    # X-Axis Mag MSB, X-Axis Mag LSB
      data0 = bus.read_byte_data(0x1E, 0x03)
      data1 = bus.read_byte_data(0x1E, 0x04)

    # Convert the data
      xMag = data0 * 256 + data1
      if xMag > 32767 :
            xMag -= 65536

    # LSM303DLHC Mag address, 0x1E(30)
    # Read data back from 0x05(05), 2 bytes
    # Y-Axis Mag MSB, Y-Axis Mag LSB
      data0 = bus.read_byte_data(0x1E, 0x07)
      data1 = bus.read_byte_data(0x1E, 0x08)

    # Convert the data
      yMag = data0 * 256 + data1
      if yMag > 32767 :
            yMag -= 65536

    # LSM303DLHC Mag address, 0x1E(30)
    # Read data back from 0x07(07), 2 bytes
    # Z-Axis Mag MSB, Z-Axis Mag LSB
      data0 = bus.read_byte_data(0x1E, 0x05)
      data1 = bus.read_byte_data(0x1E, 0x06)

    # Convert the data
      zMag = data0 * 256 + data1
      if zMag > 32767 :
            zMag -= 65536

      return (xMag,yMag,zMag)



class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        print("Loading UI")
        file = QFile(os.path.abspath(os.path.dirname(sys.argv[0]))+"/charlotte.ui")
        file.open(QFile.ReadOnly)
        loader = QUiLoader(self)
        self.w = loader.load(file)
        self.setCentralWidget(self.w)
        self.setWindowTitle("Charlotte")
        self.w.save.clicked.connect(self.SaveSurvey)
        self.w.selectcave.clicked.connect(self.OpenSurvey)
        self.w.puntofisso.clicked.connect(self.puntofisso)
        self.w.openlastcave.stateChanged.connect(self.EditConf)
        self.w.exit.clicked.connect(self.chiudi)
        self.w.shutdown.clicked.connect(self.shutdown)
        self.w.reboot.clicked.connect(self.reboot)
        self.w.accesspoint.clicked.connect(self.accesspoint)
        self.w.touch_calibrate.clicked.connect(self.touch_calibrate)
        self.w.deleterow.clicked.connect(self.deleterow)
        self.w.deletefixrow.clicked.connect(self.deletefixrow)
        self.w.savetable.clicked.connect(self.savetable)
        self.w.updatedrawing.clicked.connect(self.updatedrawing)
        self.w.updatedxf.clicked.connect(self.updatedxf)
        self.w.updatelidarsvg.clicked.connect(self.updatelidarsvg)
        self.w.readGPS.clicked.connect(self.readGPS)
        self.w.getGPStime.clicked.connect(self.getGPStime)
        self.w.showDeviceInfo.clicked.connect(self.showDeviceInfo)
        self.w.zoom.valueChanged.connect(self.zoomDrawings)
        self.w.setDeclination.clicked.connect(self.setDeclination)
        self.w.calibraBussola.clicked.connect(self.calibraBussola)
        self.w.setMsl.clicked.connect(self.setMsl)
        self.w.setAlt.clicked.connect(self.setAlt)
        #self.w.tempImpostata.valueChanged.connect(self.setTempImp)
        self.w.piantacombo.currentTextChanged.connect(self.piantacombo)
        self.w.spaccatocombo.currentTextChanged.connect(self.spaccatocombo)
        self.requiredData = {}
        self.section = [0.0 for deg in range(360)]  #we expect to get one value for every degree
        self.walls = {}
        self.mycfgfile = QDir.homePath() + "/.charlottecfg"
        self.myCaveFile = {}
        self.myCoordinates = {}
        self.branches = []
        self.mycfg = {}
        self.CompassCalibration = None
        self.newCave()
        #Take note that Y axis on a graphicsview is flipped
        #self.w.pianta.scale(1,-1)
        #self.w.spaccato.scale(1,-1)
        #self.w.section.scale(1,-1)
        self.w.pianta.scale(1,1)
        self.w.spaccato.scale(1,1)
        self.w.section.scale(1,1)
        self.w.section.scale(40,40)
        self.csvheader = ["From", "To", "SideTilt", "FrontalInclination", "heading", "distance", "left", "right", "up", "down", "latitude", "longitude", "altitude", "notes"]
        print("UI loaded")
        QApplication.processEvents()
        self.msl = 1013.0
        self.loadPersonalCFG()
        if os.path.isfile(self.mycfg['lastcave']) and self.mycfg['startfromlastcave']=='True':
            self.openFile()
        self.sezioneScene = QGraphicsScene()
        if isRPI:
            self.getDataThread = getData(self)
            self.getDataThread.GotScan.connect(self.LidarScanDone)
            self.getDataThread.CompassCal.connect(self.CompasCalDone)
            self.getDataThread.msl = self.msl
            self.getDataThread.start()
        #self.startLidarScan()
        self.firstdistance = 0.0
        self.th = threading.Thread(target=self.puntofissoSave) #args=self.firstdistance
        self.th.start()
        self.currentzoom = 1
        self.listCaves()
        if isRPI:
            self.w.dxfmesh.setChecked(False)
        self.showDeviceInfo()
        self.compassdialog = QDialog(self)
        self.compassdialoglbl = QLabel("Starting compass calibration")
        self._projections = {}


    #TODO: eventfilter for keypad https://stackoverflow.com/questions/27113140/qt-keypress-event-on-qlineedit

    def showDeviceInfo(self):
        fulltext = "Informazioni sul dispositivo:\n"
        if isWindows:
            lines = "Windows"
        else:
            os.system("ip -br a > /tmp/devinfo.txt")
            os.system("date >> /tmp/devinfo.txt")
            text_file = open("/tmp/devinfo.txt", "r", encoding='utf-8')
            lines = text_file.read()
            text_file.close()
        self.w.deviceinfo.setText(fulltext+lines)

    def OpenSurvey(self):
        self.mycfg['lastcave'] = self.w.cavename.currentText()
        self.openFile()
        self.savePersonalCFG()

    def chiudi(self):
        sys.exit(0)

    def shutdown(self):
        if isRPI:
            os.system('sudo halt')

    def reboot(self):
        if isRPI:
            os.system('sudo reboot')

    def accesspoint(self):
        if isRPI:
            print("Access Point mode")
            os.system(os.path.abspath(os.path.dirname(sys.argv[0]))+'/accesspoint.sh')
            self.w.statusbar.showMessage("Please connect to charlotte.lan or 192.168.1.1")
            #attiviamo la scheda wifi
            #abilitiamo un samba share sulla cartella dei rilievi
            #abilitiamo un server web con listing dei file sulla cartella dei rilievi
            #diciamo all'utente di connettersi a 192.168.1.1

    def touch_calibrate(self):
        if isRPI:
            print("Touchscreen calibration")
            tmpfilename = "/tmp/xinput_calibrator.txt"
            conffile = "/usr/share/X11/xorg.conf.d/40-libinput.conf"
            os.system("xinput_calibrator > "+tmpfilename)
            #https://www.waveshare.com/wiki/5inch_HDMI_LCD_(B)
            text_file = open(tmpfilename, "r", encoding='utf-8')
            lines = text_file.read().split("\n")
            text_file.close()
            newcal = []
            for l in range(len(lines)):
                if "\"InputClass\"" in lines[l]:
                    newcal.append(lines[l])
                    newcal.append(lines[l+1])
                    newcal.append(lines[l+2])
                    newcal.append(lines[l+3])
                    newcal.append(lines[l+4])
                    newcal.append(lines[l+5])
                    break
            text_file = open(conffile, "r", encoding='utf-8')
            lines = text_file.read().split("\n")
            text_file.close()
            found = False
            for l in range(len(lines)):
                if "WaveShare" in lines[l]:
                    lines[l+1] = newcal[3] #Option Calibration
                    found = True
                    break
            if not found:
                lines.extend(newcal)
            conf = ""
            for line in lines:
                conf = conf + line + "\n"
            tmpconf = "/tmp/touch_calibrate-xorg.conf"
            text_file = open(tmpconf, "w", encoding='utf-8')
            text_file.write(conf)
            text_file.close()
            os.system("sudo cp "+conffile+ " " +conffile+"."+datetime.now().strftime("%Y%m%d%H%M%S"))
            os.system("sudo mv "+tmpconf+ " " +conffile)
            self.w.statusbar.showMessage("Calibration values written in /usr/share/X11/xorg.conf.d/40-libinput.conf")
            
    def calibraBussola(self):
        #https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration?view=all
        #https://github.com/adafruit/Adafruit_LSM303DLH_Mag/blob/master/examples/calibration/calibration.ino
        #https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
        #https://www.instructables.com/Configure-read-data-calibrate-the-HMC5883L-digital/
        if not isRPI:
            print("Unable to get compass if not on RaspberryPi")
            #return
        print("For high precision calibration, please look at this software: https://web.archive.org/web/20200221040955/http://www.varesano.net/blog/fabio/freeimu-magnetometer-and-accelerometer-calibration-gui-alpha-version-out")
        if self.w.calibraBussola.isChecked():
            self.compassdialog = QDialog(self)
            self.compassdialoglbl.setText("Starting compass calibration")
            layout = QVBoxLayout()
            layout.addWidget(self.compassdialoglbl)
            QBtn = QDialogButtonBox.Cancel
            buttonBox = QDialogButtonBox(QBtn)
            #buttonBox.accepted.connect(self.compassdialog.accept)
            buttonBox.accepted.connect(self.CompasCalDone)
            layout.addWidget(buttonBox)
            self.compassdialog.setLayout(layout)
            self.compassdialog.show()
            self.w.manualMode.setChecked(True)
        #print("Loading data...")
        #time.sleep(0.5)
        #QApplication.processEvents()
        #time.sleep(10)
        #lbl.setText("Sto lavorando\nFunziona")
        
    def CompasCalDone(self, calData = None):
        self.compassdialog.hide()
        print("Received compass calibration data:")
        print(calData)
        if calData != None:
            self.mycfg["CompassCalibration"] = calData
            self.savePersonalCFG()
            self.w.statusbar.showMessage("Writing configuration, please reboot Charlotte.")
        self.w.calibraBussola.setChecked(False)
        return
        
    def setAlt(self):
        alt = self.w.actualAlt.value()
        msl = self.getDataThread.calibrateBMP(alt)
        self.msl = msl
        self.mycfg["msl"] = self.msl
        self.savePersonalCFG()
        self.w.actualMsl.setValue(self.msl)
        
    def setMsl(self):
        msl = self.w.actualMsl.value()
        self.msl = msl
        self.getDataThread.msl = self.msl
        self.mycfg["msl"] = self.msl
        self.savePersonalCFG()

    def newCave(self):
        self.w.fromP.setText("0")
        self.w.toP.setText("1")

    def loadPersonalCFG(self):
        try:
            text_file = open(self.mycfgfile, "r", encoding='utf-8')
            lines = text_file.read()
            text_file.close()
            self.mycfg = json.loads(lines.replace("\n", "").replace("\r", ""))
        except:
            print("Creo il file di configurazione")
        cfgtemplate = {'outputfolder': QDir.homePath() + "/charlottedata", 'lastcave': '', 'startfromlastcave': 'True',
        'calibration': {
        'x': {'min': '', 'max':''},
        'y': {'min': '', 'max':''},
        'z': {'min': '', 'max':''},
        'distance': ''
        },
        'declination': [0,0]
        }
        for key in cfgtemplate:
            if key not in self.mycfg or len(self.mycfg[key])==0:
                self.mycfg[key] = cfgtemplate[key]
                self.savePersonalCFG()
        if not os.path.isdir(self.mycfg["outputfolder"]) and not os.path.isfile(self.mycfg["outputfolder"]):
            os.mkdir(self.mycfg["outputfolder"])
            #if Rpi:
            #    os.system('echo "Options +Indexes" >> "'+self.mycfg["outputfolder"]+'/.htaccess"')
            #TODO: check if /var/www/html is actually a symlink
            #    os.system('sudo rm /var/www/html')
            #    os.system('#    'sudo ln -s "'+self.mycfg["outputfolder"]+'" /var/www/html')
        if not os.path.isdir(self.mycfg["outputfolder"]):
            self.mycfg["outputfolder"] = cfgtemplate["outputfolder"]
            self.savePersonalCFG()
            try:
                os.mkdir(self.mycfg["outputfolder"])
            except:
                pass
        try:
            self.w.decDeg.setValue(self.mycfg['declination'][0])
            self.w.decMin.setValue(self.mycfg['declination'][1])
        except:
            pass
        try:
            self.CompassCalibration = self.mycfg['CompassCalibration']
        except:
            self.CompassCalibration = None
        try:
            self.msl = self.mycfg["msl"]
            self.w.actualMsl.setValue(self.msl)
        except:
            print("Error setting MSL value")
            pass
        self.w.outputfolder.setText(self.mycfg["outputfolder"])
        if self.mycfg["startfromlastcave"]=='True':
            self.w.openlastcave.setChecked(True)
            self.openFile()
        else:
            self.w.openlastcave.setChecked(False)

    def EditConf(self):
        #cfgtemplate = {'outputfolder': QDir.homePath() + "/charlotte", 'lastcave': '', 'startfromlastcave': '', 'sessions': [],
        self.mycfg['startfromlastcave'] = "False"
        if self.w.openlastcave.isChecked():
            self.mycfg['startfromlastcave'] = "True"
        self.savePersonalCFG()
        
    def setDeclination(self):
        self.mycfg['declination'] = [self.w.decDeg.value(),self.w.decMin.value()]
        self.savePersonalCFG()

    def savePersonalCFG(self):
        cfgtxt = json.dumps(self.mycfg)
        text_file = open(self.mycfgfile, "w", encoding='utf-8')
        text_file.write(cfgtxt)
        text_file.close()

    def itIsOff(self):
        self.w.save.setChecked(False)
        self.SaveSurvey()
        
    def setRequiredData(self, mydata):
        self.requiredData = mydata
        self.validateData()
        self.w.temperature.setValue(self.requiredData["temperature"])
        self.w.pressure.setValue(self.requiredData["pressure"])
        self.w.bar_altitude.setValue(self.requiredData["bar_altitude"])
        self.w.sideTilt.setValue(self.requiredData["sideTilt"])
        self.w.frontalInclination.setValue(self.requiredData["frontalInclination"])
        self.w.heading.setValue(self.requiredData["heading"])
        self.w.distance.setValue(self.requiredData["distance"])

    def validateData(self):
        try:
            if self.requiredData["temperature"] < -100:
                t = 0/0
        except:
            self.requiredData["temperature"] = 0.0
            #print("Error reading temperature")

    def readGPS(self):
        self.GPSThread = getGPS(self)
        self.GPSThread.gpsposition.connect(self.gotGPSpos)
        self.GPSThread.start()

    def gotGPSpos(self, gpsfix):
        print("Got GPS fix data:")
        print(gpsfix)
        self.w.latitude.setValue(gpsfix[0])
        self.w.longitude.setValue(gpsfix[1])

    def getGPStime(self):
        self.GPSThread = getGPS(self)
        self.GPSThread.gpstime.connect(self.gotGPStime)
        self.GPSThread.start()

    def gotGPStime(self, gpstime):
        print("Got GPS time data:")
        print(gpstime)

    def startLidarScan(self):
        #if isRPI:
        useLidar = True
        #useLidar = False
        if useLidar:
            self.LidarThread = getLidar(self)
            self.LidarThread.GotScan.connect(self.LidarScanDone)
            # Controlliamo se sia spento: #self.LidarThread.finished.connect(self.itIsOff)
            self.LidarThread.start()

    def LidarScanDone(self, output):
        if len(output) < 360:
            self.section = [0.0 for deg in range(360)]  #we expect to get one value for every degree
        else:
            self.section = output
            try:
                sideTilt = self.requiredData["sideTilt"]
            except:
                sideTilt = 0
            self.calculateWalls(self.section, sideTilt)
            self.drawSection()

    def getCoordinates(self, Cfile):
        coord = {}
        pointname = Cfile["measurements"][0]["from"]
        frompointname = Cfile["measurements"][0]["from"]
        myX = 0
        myY = 0
        myZ = 0
        coord[pointname] = self.getPointWalls(pointname, frompointname, myX, myY, myZ)
        for row in Cfile["measurements"]:
            pointname = row["to"]
            frompointname = row["from"]
            try:
                fromX = coord[frompointname]["pos"][0]
                fromY = coord[frompointname]["pos"][1]
                fromZ = coord[frompointname]["pos"][2]
            except:
                fromX = 0
                fromY = 0
                fromZ = 0
            try:
                sideTilt = row["topographic"]['sideTilt']
                incl = row["topographic"]['frontalInclination']
                heading = row["topographic"]['heading']
                dist = row["topographic"]['distance']
            except:
                sideTilt = 0
                incl = 0
                heading = 0
                dist = 0
            myX = fromX + (dist*(math.cos(math.radians(heading)))*(math.cos(math.radians(incl))))
            myY = fromY + (dist*(math.sin(math.radians(heading)))*(math.cos(math.radians(incl))))
            myZ = fromZ + (dist*(math.sin(math.radians(incl))))
            coord[pointname] = self.getPointWalls(pointname, frompointname, myX, myY, myZ)
        self.myCoordinates = coord

    def getPointWalls(self, pointname, frompointname, myX, myY, myZ, Cfile = None, newbranch = False):
        pcoords = {}
        if Cfile == None:
            Cfile = self.myCaveFile
        rawdata = self.getFromPointData(Cfile, frompointname, pointname)
        try:
            sideTilt = rawdata["topographic"]['sideTilt']
            incl = rawdata["topographic"]['frontalInclination']
            heading = rawdata["topographic"]['heading']
            dist = rawdata["topographic"]['distance']
        except:
            sideTilt = 0
            incl = 0
            heading = 0
            dist = 0
        if not newbranch:
            rawdata = self.getFromPointData(Cfile, pointname)
        if len(rawdata) >0:
            l = rawdata["walls"]['left']
            r = rawdata["walls"]['right']
            u = rawdata["walls"]['up']
            d = rawdata["walls"]['down']
            myCenter = [myX,myY,myZ]
            left = self.calculate3DCoord(l, 0, myCenter, heading, incl, sideTilt)
            down = self.calculate3DCoord(d, 90, myCenter, heading, incl, sideTilt)
            right = self.calculate3DCoord(r, 180, myCenter, heading, incl, sideTilt)
            up = self.calculate3DCoord(u, 270, myCenter, heading, incl, sideTilt)
            pcoords = {"pos":myCenter,"left":left,"right":right,"up":up,"down":down}
        else:
            print("Unable to getFromPointData for " +str(pointname))
            pcoords = {"pos":[myX,myY,myZ],"left":[myX,myY,myZ],"right":[myX,myY,myZ],"up":[myX,myY,myZ],"down":[myX,myY,myZ]}
        return pcoords

    #def getPointSection(self, pointname, frompointname, myX, myY, myZ, Cfile = None):
    #    print("Calculating section points")
    #    scoords = []
    #    return scoords



    def calculateSectionCoord(self, section, myCenter = [0.0, 0.0, 0.0], heading = 0.0, incl = 0.0, sidetilt = 0.0):
        if len(section) <360:
            return None
        coords = []
        for angle in range(360):
            tmpangle = 360-(angle)
            d = section[angle]
            tmpcoord = self.calculate3DCoord(d, tmpangle, myCenter, heading, incl, sidetilt)
            coords.append(tmpcoord)
        return coords

    def calculate3DCoord(self, d, angle, myCenter = [0.0, 0.0, 0.0], heading = 0.0, incl = 0.0, sidetilt = 0.0):
        #
        tmpangle = (angle-sidetilt)
        x = - d*math.cos(math.radians(tmpangle))
        y = 0
        z = d*math.sin(math.radians(tmpangle))

        #This code seems to be slower than numpy
        #rotate by incl
        #iCorr = 0.0
        #yold = y
        #zold = z
        #y = (yold*(math.cos(math.radians(incl+iCorr)))) - (zold*(math.sin(math.radians(incl+iCorr))))
        #z = (yold*(math.sin(math.radians(incl+iCorr)))) + (zold*(math.cos(math.radians(incl+iCorr))))
        #rotate by heading
        #hCorr = 90
        #xold = -x
        #yold = y
        #x = (xold*(math.cos(math.radians(heading+hCorr)))) - (yold*(math.sin(math.radians(heading+hCorr))))
        #y = (xold*(math.sin(math.radians(heading+hCorr)))) + (yold*(math.cos(math.radians(heading+hCorr))))

        #Rotate by inclination
        iCorr = 0.0
        theta = np.radians(-incl+iCorr)
        #rotation matrix
        r = np.array((
         (1,0,0),
         (0,np.cos(theta),-np.sin(theta)),
         (0,np.sin(theta),np.cos(theta))
        ))
        #Vector X,Y,Z
        v = np.array((x,y,z))
        #apply the rotation matrix r to v: r*v
        newCoords = r.dot(v)
        x = newCoords[0]
        y = newCoords[1]
        z = newCoords[2]

        #Rotate by heading
        hCorr = 90 #section should be perpendicular to the heading
        theta = np.radians(heading+hCorr)
        #rotation matrix
        r = np.array((
         (np.cos(theta),-np.sin(theta), 0),
         (np.sin(theta),np.cos(theta), 0),
         (0,0,1)
        ))
        #Vector X,Y,Z
        v = np.array((x,y,z))
        #dot product matrix*vector
        newCoords = r.dot(v)
        x = newCoords[0]
        y = newCoords[1]
        z = newCoords[2]

        #Translate by center
        x = x + myCenter[0]
        y = y + myCenter[1]
        z = z + myCenter[2]

        #add values to coords list
        coords = [x,y,z]
        return coords

    def calculateWalls(self, section, sideTilt = 0):
        #+int(sideTilt)
        leftWall = (self.averageListRange(section, 315+int(-sideTilt),360+int(-sideTilt))+self.averageListRange(section, 0+int(-sideTilt),45+int(-sideTilt)))/2
        downWall = self.averageListRange(section, 45+int(-sideTilt),135+int(-sideTilt))
        rightWall = self.averageListRange(section, 135+int(-sideTilt),225+int(-sideTilt))
        upWall = self.averageListRange(section, 225+int(-sideTilt),315+int(-sideTilt))

        self.walls = {'left':leftWall, 'right':rightWall, 'up':upWall, 'down':downWall}
        self.w.leftW.setValue(leftWall)
        self.w.rightW.setValue(rightWall)
        self.w.upW.setValue(upWall)
        self.w.downW.setValue(downWall)

    def averageListRange(self, mylist, myfrom = 0, myto = None):
        if myto == None:
            myto = len(mylist)
        myfrom = myfrom%(len(mylist)+1)
        myto = myto%(len(mylist)+1)
        sum = 0
        for el in range(myfrom, myto):
            try:
                sum = sum + float(mylist[el])
            except:
                continue
        average = sum/(myto-myfrom)
        return average

    def cleanName(self, name):
        cleanedname = re.sub("[^0-9A-Za-z\_\-]", "_", name)
        return cleanedname

    def saveFile(self, doDraw = True):
        cleanedname = self.cleanName(self.w.cavename.currentText())
        self.mycfg["lastcave"] = cleanedname
        if self.mycfg["lastcave"] == "":
            print("You need to specify a name for this cave")
            return
        self.savePersonalCFG()

        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        if not os.path.isdir(cavefolder):
            os.mkdir(cavefolder)
        Cfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".json"
        Tfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".csv"
        Xfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".csx"
        Dfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".DAT"
        print("Saving to " + Cfilename)
        
        try:
            now = datetime.now()
            nowts = now.strftime("%d%m%Y%H%M%S")
            bckdir = os.path.dirname(Cfilename)
            bckfile = os.path.basename(Cfilename)+"-bck"+nowts
            try:
                os.makedirs(bckdir+"/backups/")
            except Exception as e:
                #print(e)
                pass
            shutil.copy(Cfilename, bckdir+"/backups/"+bckfile)
            print("Backup in "+bckdir+"/backups/"+bckfile)
        except:
            print("Error during backup")
            pass
        cfiletxt = json.dumps(self.myCaveFile).replace(",",",\n")
        text_file = open(Cfilename, "w", encoding='utf-8')
        text_file.write(cfiletxt)
        text_file.close()

        #now we build the CSV based on the measurements
        tfiletxt = self.json2CSV(self.myCaveFile)
        text_file = open(Tfilename, "w", encoding='utf-8')
        text_file.write(tfiletxt)
        text_file.close()
        self.populateTable(tfiletxt)

        #now we build the cSurvey CSX file
        xfiletxt = self.json2CSX(self.myCaveFile)
        text_file = open(Xfilename, "w", encoding='utf-8')
        text_file.write(xfiletxt)
        text_file.close()
        
        #now we buil the Compass DAT file
        dfiletxt = self.json2DAT(self.myCaveFile)
        text_file = open(Dfilename, "w", encoding='utf-8')
        text_file.write(dfiletxt)
        text_file.close()

        #we draw the result
        if doDraw and len(self.myCaveFile['measurements'])<2:
            self.updatedrawing()
            
    def appendNewPoint(self, firstdistance = 0.0, doDraw = True):
        cleanedname = self.cleanName(self.w.cavename.currentText())
        self.mycfg["lastcave"] = cleanedname
        if self.mycfg["lastcave"] == "":
            print("You need to specify a name for this cave")
            return
        self.savePersonalCFG()

        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        if not os.path.isdir(cavefolder):
            os.mkdir(cavefolder)
        Cfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".json"
        
        temp = self.w.temperature.value()
        press = self.w.pressure.value()
        bar_alt = self.w.bar_altitude.value()

        dist = float(self.w.distance.value())
        if firstdistance > 0.0:
            dist = firstdistance - dist
        #
        try:
            lat = self.w.latitude.value()
            lon = self.w.longitude.value()
            alt = self.w.altitude.value()
        except:
            lat = 0.0
            lon = 0.0
            alt = 0.0
        requiredData = {
        'sideTilt':self.w.sideTilt.value(),
        'frontalInclination':self.w.frontalInclination.value(),
        'heading':self.w.heading.value(),
        'distance':dist
        }

        walls = {'left':self.w.leftW.value(), 'right':self.w.rightW.value(), 'up':self.w.upW.value(), 'down':self.w.downW.value()}

        Cfiletemplate = {'caveName':self.w.cavename.currentText(), 'measurements':[]}
        Cfile = self.openJson(Cfilename)
        print(Cfile)
        if len(Cfile)==0:
            print("init file")
            Cfile = Cfiletemplate

        now = datetime.now()
        thismeasure = {
        'timestamp': now.strftime("%d/%m/%Y %H:%M:%S"),
        'from': self.w.fromP.text(),
        'to': self.w.toP.text(),
        'GPS':{'latitude': lat, 'longitude': lon, 'altitude': alt},
        'topographic': requiredData,
        'walls': walls,
        'section': self.section,
        'ambient':{'temperature': temp, 'pressure': press, 'bar_altitude': bar_alt},
        'notes': ""
        }

        Cfile['measurements'].append(thismeasure)

        self.myCaveFile = Cfile
        self.saveFile(doDraw)

    def json2CSV(self, Cfile):
        csvtxt = ""
        for row in Cfile["measurements"]:
            csvtxt = csvtxt + str(row["from"]) + ","
            csvtxt = csvtxt + str(row["to"]) + ","

            csvtxt = csvtxt + str(row["topographic"]['sideTilt']) + ","
            csvtxt = csvtxt + str(row["topographic"]['frontalInclination']) + ","
            csvtxt = csvtxt + str(row["topographic"]['heading']) + ","
            csvtxt = csvtxt + str(row["topographic"]['distance']) + ","

            csvtxt = csvtxt + str(row["walls"]['left']) + ","
            csvtxt = csvtxt + str(row["walls"]['right']) + ","
            csvtxt = csvtxt + str(row["walls"]['up']) + ","
            csvtxt = csvtxt + str(row["walls"]['down']) + ","

            try:
                csvtxt = csvtxt + str(row["GPS"]['latitude']) + ","
            except:
                csvtxt = csvtxt + "0.0,"
            try:
                csvtxt = csvtxt + str(row["GPS"]['longitude']) + ","
            except:
                csvtxt = csvtxt + "0.0,"
            try:
                csvtxt = csvtxt + str(row["GPS"]['altitude']) + ","
            except:
                csvtxt = csvtxt + "0.0,"

            try:
                csvtxt = csvtxt + str(row["notes"]) + ","
            except:
                csvtxt = csvtxt + ","

            #csvtxt = csvtxt + str(row["ambient"]['temperature']) + ","
            #csvtxt = csvtxt + str(row["ambient"]['pressure']) + ","

            #for num in row["section"]:
            #   csvtxt = csvtxt + str(num) + ";"
            #csvtxt = csvtxt + ","

            csvtxt = csvtxt[0:-1] + "\n"
        return csvtxt

    def json2CSX(self, Cfile):
        cavename = self.w.cavename.currentText()
        now = datetime.now()
        mydate = now.strftime("%d-%m-%Y")
        #header
        #TODO: set origin
        csxtxt = "<csurvey version=\"1.12\" id=\"a6278b65-2d7a-4708-ad16-3693726b7e94\">\n  <properties id=\"\" name=\"\" creatid=\"cSurvey\" creatversion=\"1.12\" creatdate=\""+now.strftime("%Y-%m-%d")+"T05:39:28.8212330-07:00\" creat_postprocessed=\"1\" origin=\"REGINA1_0\" calculatemode=\"1\" calculatetype=\"2\" calculateversion=\"3\" ringcorrectionmode=\"2\" nordcorrectionmode=\"0\" inversionmode=\"1\" designwarpingmode=\"1\" bindcrosssection=\"1\" threedlochshowsplay=\"0\" slpeo=\"1\">\n    <sessions>\n      <session date=\""+now.strftime("%Y-%m-%d")+"T00:00:00.0000000\" description=\""+cavename+"\" team=\"?\" depthtype=\"0\" nordtype=\"0\" declinationenabled=\"0\" declination=\"0.00\" sidemeasuresreferto=\"1\" />\n    </sessions>\n"
        #caveinfo
        csxtxt = csxtxt + "    <caveinfos>\n      <caveinfo id=\"\" name=\""+cavename.upper()+"\">\n        <branches>\n          <branch name=\"regina1\">\n            <branches>\n              <branch name=\"regina1\">\n                <branches />\n              </branch>\n            </branches>\n          </branch>\n        </branches>\n      </caveinfo>\n    </caveinfos>\n    <cavevisibilityprofiles />\n    <gps refpointonorigin=\"1\" format=\"\" />\n    <designproperties />\n    <datatables>\n      <segments>\n        <datafield name=\"import_source\" type=\"0\" />\n        <datafield name=\"import_date\" type=\"5\" />\n      </segments>\n      <trigpoints />\n      <designitems />\n    </datatables>\n    <hlsds>\n      <hlsd id=\"_ring\" n=\"Loops\" colors=\"-16776961\" sz=\"1.67\" at=\"1\" cnd=\"\" sys=\"1\" />\n      <hlsd id=\"_entrance\" n=\"Entrance\" colors=\"-65536\" at=\"0\" cnd=\"vb#&gt;Details.Element.isentrance\" sys=\"1\" />\n      <hlsd id=\"_exploration\" n=\"Points need exploration\" colors=\"-7278960\" at=\"0\" cnd=\"vb#&gt;Details.Element.isinexploration\" sys=\"1\" />\n      <hlsd id=\"_gpsdefaultfix\" n=\"Stations with GPS coordinates\" colors=\"-16751616\" at=\"0\" cnd=\"vb#&gt;not Details.Element.coordinate.isempty andalso Details.Element.coordinate.fix=0\" sys=\"1\" />\n      <hlsd id=\"_gpsmanualfix\" n=\"Stations with GPS coordinates (manual fix)\" colors=\"-7357301\" at=\"0\" cnd=\"vb#&gt;not Details.Element.coordinate.isempty andalso Details.Element.coordinate.fix=1\" sys=\"1\" />\n      <hlsd id=\"_shotwithnote\" n=\"Stations with note\" colors=\"-10496\" at=\"0\" cnd=\"vb#&gt;Details.Element.note&lt;&gt;&quot;&quot;\" sys=\"1\" />\n      <hlsd id=\"_stationwithnote\" n=\"Segments with note\" colors=\"-10496\" at=\"1\" cnd=\"vb#&gt;Details.Element.note&lt;&gt;&quot;&quot;\" sys=\"1\" />\n      <hlsd id=\"_stationbyalt\" n=\"Stations by altitude\" colors=\"-8355712\" at=\"0\" cnd=\"vb#&gt;&gt;public function GetHighlight(Details as cStationHighlightDetails) as boolean&#xD;&#xA;Details.meters.color = Survey.calculate.trigpoints.zs.GetScaleColor(Survey.calculate.trigpoints(Details.element).point.z)&#xD;&#xA;Return True&#xD;&#xA;end function\" sys=\"1\" />\n      <hlsd id=\"_shotbyalt\" n=\"Segments by altitude\" colors=\"-8355712\" at=\"1\" cnd=\"vb#&gt;&gt;public function GetHighlight(Details as cShotHighlightDetails) as boolean&#xD;&#xA;Details.meters.colors = {Survey.calculate.trigpoints.zs.GetScaleColor(Survey.calculate.trigpoints(Details.element.getfromtrigpoint).point.z), Survey.calculate.trigpoints.zs.GetScaleColor(Survey.calculate.trigpoints(Details.element.gettotrigpoint).point.z)}&#xD;&#xA;Return True&#xD;&#xA;end function\" sys=\"1\" />\n    </hlsds>\n  </properties>\n  <attachments />"
        csxtxt = csxtxt + "\n<segments>"

        i = 0
        s = 5
        for row in Cfile["measurements"]:
            mydate = str(row["timestamp"].split(" ")[0]).replace("/", "")
            
            mybranch = "None"
            for b in range(len(self.branches)):
                if row["to"] in self.branches[b]:
                    mybranch = str(b)

            #csvtxt = csvtxt + str(row["topographic"]['sideTilt']) + ","
            
            segFrom = cavename.upper() + "_" + str(row["from"])
            segTo = cavename.upper() + "_" + str(row["to"])

            csxtxt = csxtxt + "<segment id=\"bf52f3d0-7450-4998-b28b-e0163ac3c4ae\" from=\""+segFrom+"\" to=\""+segTo+"\" distance=\"1.43\" inclination=\"-30.48\" bearing=\"40.42\" l=\"0.48\" r=\"1.00\" u=\"0.35\" d=\"0.05\" cave=\"regina1\" branch=\"regina1\regina1\" session=\"20220503_regina1\" plansplayborderinclinationrange=\"-90.0;90.0\" profilesplayborderposinclinationrange=\"0.0;90.0\" profilesplayborderneginclinationrange=\"-90.0;0.0\">\n      <data>\n        <srcdata st=\""+segTo+"\" sf=\""+segFrom+"\" d=\"1.43\" i=\"-30.48\" b=\"40.42\" dr=\"0\" />\n"
            csxtxt = csxtxt + "            <olddata st=\""+segTo+"\" sf=\""+segFrom+"\" d=\"0\" i=\"0\" b=\"0\" dr=\"0\" />\n        <data st=\""+segTo+"\" sf=\""+segFrom+"\" d=\"1.430\" i=\"-30.479\" b=\"40.3864969847901\" dr=\"0\" />\n        <planpd c=\"1\" of=\"\" ot=\"\" ofpx=\"0\" ofpy=\"0\" otpx=\"0\" otpy=\"0\" ofsprx=\"0\" ofspry=\"0\" ofsplx=\"0\" ofsply=\"0\" otsprx=\"0\" otspry=\"0\" otsplx=\"0\" otsply=\"0\" ofbr=\"0\" ofbl=\"0\" otbr=\"0\" otbl=\"0\" f=\""+segFrom+"\" t=\""+segTo+"\" fpx=\"0\" fpy=\"0\" tpx=\"0.7986\" tpy=\"-0.9388\" fsprx=\"0.761691\" fspry=\"0.6479404\" fsplx=\"-0.36561169447124\" fsply=\"-0.311011396681647\" tsprx=\"1.4079528\" tspry=\"-0.4204477\" tsplx=\"0.623411063065864\" tsply=\"-1.087826294243289\" fbr=\"130.3864969847901\" fbl=\"310.3864969847901\" tbr=\"130.3864969847901\" tbl=\"310.3864969847901\">\n          <fspds fp=\"0\" />\n          <tspds fp=\"0\" />\n        </planpd>\n"
            csxtxt = csxtxt + "            <profilepd c=\"1\" of=\"\" ot=\"\" ofpx=\"0\" ofpy=\"0\" otpx=\"0\" otpy=\"0\" ofspux=\"0\" ofspuy=\"0\" ofspdx=\"0\" ofspdy=\"0\" otspux=\"0\" otspuy=\"0\" otspdx=\"0\" otspdy=\"0\" f=\""+segFrom+"\" t=\""+segTo+"\" fpx=\"0\" fpy=\"0.0000\" tpx=\"1.23252075033242\" tpy=\"0.7254\" fspux=\"0\" fspuy=\"-0.3500\" fspdx=\"0\" fspdy=\"0.0500\" tspux=\"1.23252075033242\" tspuy=\"0.3154\" tspdx=\"1.23252075033242\" tspdy=\"1.1754\">\n          <fspds fp=\"0\" />\n          <tspds fp=\"0\" />\n          <surfaceprofilepd />\n        </profilepd>\n        <sds />\n      </data>\n      <attachments />\n      <datarow>compass|2022-05-05T05:45:27.1944770-07:00</datarow>\n    </segment>"
            
            csxtxt = csxtxt + "<segment id=\""+str(s)+"\" cave=\""+cavename.upper()+"\" branch=\""+mybranch+"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["to"]) +"\" distance=\""+ str(row["topographic"]['distance']) +"\" bearing=\""+ str(row["topographic"]['heading']) +"\" inclination=\""+ str(row["topographic"]['frontalInclination']) +"\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\""+ str(row["walls"]['left']) +"\" r=\""+ str(row["walls"]['right']) +"\" u=\""+ str(row["walls"]['up']) +"\" d=\""+ str(row["walls"]['down']) +"\" distox=\"\" >\n    </segment>"
            s = s + 5

        #end segments
        csxtxt= csxtxt + "</segments>"
        #footer
        csxtxt= csxtxt + "<trigpoints> \n  </trigpoints> \n  <plan> \n    <layers> \n      <layer name=\"Base\" type=\"0\"> \n         <items /> \n      </layer> \n      <layer name=\"Soil\" type=\"1\"> \n        <items /> \n      </layer> \n      <layer name=\"Water and floor morphologies\" type=\"2\"> \n        <items /> \n      </layer> \n      <layer name=\"Rocks and concretions\" type=\"3\"> \n        <items /> \n      </layer> \n      <layer name=\"Ceiling morphologies\" type=\"4\"> \n        <items /> \n      </layer> \n      <layer name=\"Borders\" type=\"5\"> \n        <items> \n        </items> \n      </layer> \n      <layer name=\"Signs\" type=\"6\"> \n        <items /> \n      </layer> \n    </layers> \n    <plot /> \n  </plan> \n  <profile> \n    <layers> \n      <layer name=\"Base\" type=\"0\"> \n         <items /> \n      </layer> \n      <layer name=\"Soil\" type=\"1\"> \n        <items /> \n      </layer> \n      <layer name=\"Water and floor morphologies\" type=\"2\"> \n        <items /> \n      </layer> \n      <layer name=\"Rocks and concretions\" type=\"3\"> \n        <items /> \n      </layer> \n      <layer name=\"Ceiling morphologies\" type=\"4\"> \n        <items /> \n      </layer> \n      <layer name=\"Borders\" type=\"5\"> \n        <items> \n        </items> \n      </layer> \n      <layer name=\"Signs\" type=\"6\"> \n        <items /> \n      </layer> \n    </layers> \n    <plot /> \n  </profile> \n</csurvey>"
        return csxtxt
    
    def json2DAT(self, Cfile):
        dattxt = ""
        #https://www.fountainware.com/compass/Documents/ItalianTutorial.pdf
        dattxt = dattxt + Cfile["caveName"] + "\r\n"
        dattxt = dattxt + "SURVEY NAME: " + Cfile["caveName"] + "\r\n"
        dattxt = dattxt + "SURVEY DATE: 5 3 2022  COMMENT:"+ "\r\n"
        dattxt = dattxt + "SURVEY TEAM: "+ "\r\n"
        dattxt = dattxt + "?"+ "\r\n"
        dattxt = dattxt + "DECLINATION:    0.00  FORMAT: DMMDUDRLLAaDdNF  CORRECTIONS:  0.00 0.00 0.00  CORRECTIONS2:  0.00 0.00"+ "\r\n"
        dattxt = dattxt + "\r\n"
        dattxt = dattxt + "                FROM                   TO   LENGTH  BEARING      INC     LEFT       UP     DOWN    RIGHT   FLAGS  COMMENTS"+ "\r\n"
        #Origine
        dattxt = dattxt + str(Cfile["caveName"][:15] + "_0").rjust(20)
        dattxt = dattxt + str(Cfile["caveName"][:15] + "_0").rjust(20)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + str("0.00").rjust(9)
        dattxt = dattxt + "\r\n"
        #Compass si aspetta i dati in piedi e non i metri
        for row in Cfile["measurements"]:
            #dattxt = dattxt + "             "
            dattxt = dattxt + str(Cfile["caveName"][:15] + "_" + row["from"]).rjust(20)
            #dattxt = dattxt + "              "
            dattxt = dattxt + str(Cfile["caveName"][:15] + "_" + row["to"]).rjust(20)

            #dattxt = dattxt + "     8.20"
            dattxt = dattxt + str("{:.2f}".format(self.toFeet(row["topographic"]['distance']))).rjust(9)
            #dattxt = dattxt + "   340.00"
            dattxt = dattxt + str("{:.2f}".format(row["topographic"]['heading'])).rjust(9)
            #dattxt = dattxt + "   -10.00"
            dattxt = dattxt + str("{:.2f}".format(row["topographic"]['frontalInclination'])).rjust(9)
            
            #dattxt = dattxt + "    13.12"
            dattxt = dattxt + str("{:.2f}".format(self.toFeet(row["walls"]['left']))).rjust(9)
            #dattxt = dattxt + "     3.28"
            dattxt = dattxt + str("{:.2f}".format(self.toFeet(row["walls"]['right']))).rjust(9)
            #dattxt = dattxt + "     6.56"
            dattxt = dattxt + str("{:.2f}".format(self.toFeet(row["walls"]['up']))).rjust(9)
            #dattxt = dattxt + "     9.84"
            dattxt = dattxt + str("{:.2f}".format(self.toFeet(row["walls"]['down']))).rjust(9)

            #try:
            #    dattxt = dattxt + str(row["notes"])
            #except:
            #    pass

            dattxt = dattxt + "\r\n"
        dattxt = dattxt + "\x0C\x0D\x0A\x1A"
        return dattxt
    
    def toFeet(self, meters):
        try:
            feet = float(meters)*3.28084
        except:
            feet = 0.0
        return feet

    def Json2Svg(self, Cfile):
        print("Drawing SVG")
        #scenes
        spaccatoXZ = QGraphicsScene()
        spaccatoYZ = QGraphicsScene()
        pianta = QGraphicsScene()
        labelFont = QFont("Arial", 1)
        
        try:
            import simplekml
            import pyproj
            self.dokml = True
        except:
            self.dokml = False

        if self.dokml:
            kmlDoc = simplekml.Kml()
            try:
                kmlFirstPoint = (Cfile["measurements"][0]["GPS"]["latitude"],Cfile["measurements"][0]["GPS"]["longitude"])
            except:
                kmlFirstPoint = (0.0,0.0)
            print("First point GPS position: ", kmlFirstPoint)
            kmlZ, kmlL, kmlX0, kmlY0 = self.UTMproject(tuple(reversed(kmlFirstPoint)))
            pnt = kmlDoc.newpoint(name='Ingresso')
            pnt.coords = [(kmlX0, kmlY0)]
        else:
            kmlZ, kmlL, kmlX0, kmlY0 = (0,0,0,0)
            
        #Probabilmente è troppo complicato calcolare l'asse esatto https://www.speleo.it/site/images/catasto_grotte/scheda_3_ssi_normativa_rid.pdf
        lunghezza = 0.0  #Saranno inclusi i pozzi
        lunghezzarami = {}
        dislivello = 0.0
        dislivelloBar = 0.0
        altMin = 10000.0
        altMax = -10000.0

        branchID = 0
        for branch in self.branches:
            branchID = branchID + 1
            Ppoligon = QPainterPath()
            SYZpoligon = QPainterPath()
            SXZpoligon = QPainterPath()
            Ppath = QPainterPath()
            SYZpath = QPainterPath()
            SXZpath = QPainterPath()
            left = []
            right = []
            up = []
            down = []
            Ppath.moveTo(QPointF(0, 0))
            SYZpath.moveTo(QPointF(0, 0))
            SXZpath.moveTo(QPointF(0, 0))
            #tmpScnText = pianta.addText(str("0"), labelFont)
            kmlcoords = []
            kmlpoligon = []
            for branchEl in range(len(branch)):
                point = branch[branchEl]
                #print(point)
                myCoords = self.myCoordinates[point]
                #if branchEl == 0:
                #    nextpoint = branch[branchEl+1]
                #    tmpcoords = self.getPointWalls(nextpoint, point, myX, myY, myZ)
                #    pass
                myX = myCoords["pos"][0]
                myY = myCoords["pos"][1]
                myZ = myCoords["pos"][2]
                try:
                    if myZ < altMin:
                        altMin = myZ
                    if myZ > altMax:
                        altMax = myZ
                    dislivello = altMax - altMin
                except:
                    pass
                if branchEl == 0:
                    print("First point for this branch: "+ str(point))
                    nextpoint = branch[branchEl+1]
                    tmpcoords = self.getPointWalls(nextpoint, point, myX, myY, myZ, self.myCaveFile, True)
                    myCoords = tmpcoords
                    #
                    Ppath.moveTo(QPointF(myX, myY))
                    SYZpath.moveTo(QPointF(myY, -myZ))
                    SXZpath.moveTo(QPointF(myX, -myZ))
                if branchEl == len(branch)-1:
                    print("Last point for this branch: "+ str(point))
                Ppath.lineTo(QPointF(myX, myY))
                SYZpath.lineTo(QPointF(myY, -myZ))
                SXZpath.lineTo(QPointF(myX, -myZ))
                rotPoint = self.rotateNorth((myX,myY))
                kmlX = kmlX0 + rotPoint[0]
                kmlY = kmlY0 - rotPoint[1]
                if self.dokml:
                    kmlcoords.append(self.UTMunproject(kmlZ, kmlL, kmlX, kmlY))
                #Labels
                lblMargin = 4
                tmpPScnText = pianta.addText(str(point), labelFont)
                tmpPScnText.setPos(QPointF((myX-lblMargin), (myY-lblMargin)))
                tmpSYZScnText = spaccatoYZ.addText(str(point), labelFont)
                tmpSYZScnText.setPos(QPointF((myY-lblMargin), -(myZ+lblMargin)))
                tmpSXZScnText = spaccatoXZ.addText(str(point), labelFont)
                tmpSXZScnText.setPos(QPointF((myX-lblMargin), -(myZ+lblMargin)))
                # #print(Ppath.currentPosition())
                lX = myCoords["left"][0]
                lY = myCoords["left"][1]
                lZ = myCoords["left"][2]
                rX = myCoords["right"][0]
                rY = myCoords["right"][1]
                rZ = myCoords["right"][2]
                uX = myCoords["up"][0]
                uY = myCoords["up"][1]
                uZ = myCoords["up"][2]
                dX = myCoords["down"][0]
                dY = myCoords["down"][1]
                dZ = myCoords["down"][2]
                left.append([lX,lY,lZ])
                right.append([rX,rY,lZ])
                up.append([uX,uY,uZ])
                down.append([dX,dY,dZ])
            Ppoligon.moveTo(QPointF(left[0][0], left[0][1]))
            rotPoint = self.rotateNorth((left[0][0], left[0][1]))
            kmlX = kmlX0 + rotPoint[0]
            kmlY = kmlY0 - rotPoint[1]
            if self.dokml:
                kmlpoligon.append(self.UTMunproject(kmlZ, kmlL, kmlX, kmlY))
            for p in left:
                c1 = Ppoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Ppoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
                rotPoint = self.rotateNorth((p[0], p[1]))
                kmlX = kmlX0 + rotPoint[0]
                kmlY = kmlY0 - rotPoint[1]
                if self.dokml:
                    kmlpoligon.append(self.UTMunproject(kmlZ, kmlL, kmlX, kmlY))
            right.reverse()
            right.append(left[0]) #close the line
            right = right[1:]
            for p in right:
                c1 = Ppoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Ppoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
                rotPoint = self.rotateNorth((p[0], p[1]))
                kmlX = kmlX0 + rotPoint[0]
                kmlY = kmlY0 - rotPoint[1]
                if self.dokml:
                    kmlpoligon.append(self.UTMunproject(kmlZ, kmlL, kmlX, kmlY))
            if self.dokml:
                lin = kmlDoc.newlinestring(name="Poligonale-"+str(branchID), description="Poligonale del ramo "+str(branchID), coords=kmlcoords)
                lin.style.linestyle.color = "ff0000ff"
                lin.style.linestyle.width = 2
                pol = kmlDoc.newpolygon(name='Pareti')
                pol.outerboundaryis = kmlpoligon
                pol.style.linestyle.color = simplekml.Color.black
                pol.style.linestyle.width = 2
                pol.style.polystyle.color = simplekml.Color.changealphaint(50, simplekml.Color.white)
            SYZpoligon.moveTo(QPointF(up[0][1], -up[0][2]))
            SXZpoligon.moveTo(QPointF(up[0][0], -up[0][2]))
            for p in up:
                c1 = SYZpoligon.currentPosition()
                c2 = QPointF(p[1], -p[2])
                SYZpoligon.cubicTo(c1, c2, QPointF(p[1], -p[2]))
                c1 = SXZpoligon.currentPosition()
                c2 = QPointF(p[0], -p[2])
                SXZpoligon.cubicTo(c1, c2, QPointF(p[0], -p[2]))
            down.reverse()
            down.append(up[0]) #close the line
            down = down[1:]
            for p in down:
                c1 = SYZpoligon.currentPosition()
                c2 = QPointF(p[1], -p[2])
                SYZpoligon.cubicTo(c1, c2, QPointF(p[1], -p[2]))
                c1 = SXZpoligon.currentPosition()
                c2 = QPointF(p[0], -p[2])
                SXZpoligon.cubicTo(c1, c2, QPointF(p[0], -p[2]))
            PennaBordo = QPen(Qt.black, 0.1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin) #Qt.Dashline https://doc.qt.io/qtforpython/PySide2/QtGui/QPen.html
            pianta.addPath(Ppoligon, PennaBordo)
            spaccatoYZ.addPath(SYZpoligon, PennaBordo)
            spaccatoXZ.addPath(SXZpoligon, PennaBordo)
            PennaPoligonale = QPen(Qt.red, 0.1)
            pianta.addPath(Ppath, PennaPoligonale)
            spaccatoYZ.addPath(SYZpath, PennaPoligonale)
            spaccatoXZ.addPath(SXZpath, PennaPoligonale)
        #scale and north
        pBounding = pianta.itemsBoundingRect().getCoords()
        sxzBounding = spaccatoXZ.itemsBoundingRect().getCoords()
        syzBounding = spaccatoYZ.itemsBoundingRect().getCoords()
        pExtramargin = 20
        arrowSize = 2
        labelMargin = -4
        Northpath = QPainterPath()
        Northpath.moveTo(QPointF(pBounding[0]-pExtramargin, pBounding[1]-pExtramargin))
        Northpath.lineTo(QPointF(pBounding[0]-pExtramargin+(arrowSize*5), pBounding[1]-pExtramargin))
        Northpath.lineTo(QPointF(pBounding[0]-pExtramargin+(arrowSize*5), pBounding[1]-pExtramargin-arrowSize))
        Northpath.lineTo(QPointF(pBounding[0]-pExtramargin+(arrowSize*5)+arrowSize, pBounding[1]-pExtramargin))
        Northpath.lineTo(QPointF(pBounding[0]-pExtramargin+(arrowSize*5), pBounding[1]-pExtramargin+arrowSize))
        Northpath.lineTo(QPointF(pBounding[0]-pExtramargin+(arrowSize*5), pBounding[1]-pExtramargin))
        pianta.addPath(Northpath, QPen(Qt.black, 0.1))
        tmpNorthText = pianta.addText("N", QFont("Arial", 2))
        tmpNorthText.setPos(QPointF(pBounding[0]-pExtramargin+labelMargin, pBounding[1]-pExtramargin+labelMargin))
        scaleMargin = 10
        scaleWidth = 2
        scaleLevels = [0,1,2,5,10]
        biggerScale = pBounding[2]-pBounding[0]
        tmpBigScale = pBounding[3]-pBounding[1]
        if tmpBigScale > biggerScale:
            biggerScale = tmpBigScale
        tmpBigScale = sxzBounding[2]-sxzBounding[0]
        if tmpBigScale > biggerScale:
            biggerScale = tmpBigScale
        tmpBigScale = sxzBounding[3]-sxzBounding[1]
        if tmpBigScale > biggerScale:
            biggerScale = tmpBigScale
        tmpBigScale = syzBounding[2]-syzBounding[0]
        if tmpBigScale > biggerScale:
            biggerScale = tmpBigScale
        tmpBigScale = syzBounding[3]-syzBounding[1]
        if tmpBigScale > biggerScale:
            biggerScale = tmpBigScale
        for iScale in [50,100,200,500,1000]:
            if biggerScale >= (iScale*2):
                scaleLevels.append(iScale)
        PscalePoint = QPointF(pBounding[0]-pExtramargin, pBounding[1]-pExtramargin+scaleMargin)
        SXZscalePoint = QPointF(sxzBounding[0]-pExtramargin, sxzBounding[1]-pExtramargin+scaleMargin)
        SYZscalePoint = QPointF(syzBounding[0]-pExtramargin, syzBounding[1]-pExtramargin+scaleMargin)
        for scaleStep in scaleLevels:
            ScaleP = QPainterPath()
            ScaleP.moveTo(PscalePoint)
            PscalePoint = QPointF(pBounding[0]-pExtramargin+scaleStep, pBounding[1]-pExtramargin+scaleMargin)
            PscalePointDown = QPointF(pBounding[0]-pExtramargin+scaleStep, pBounding[1]-pExtramargin+scaleMargin+scaleWidth)
            ScaleP.lineTo(PscalePoint)
            ScaleP.lineTo(PscalePointDown)
            pianta.addPath(ScaleP, QPen(Qt.black, 0.1))
            tmpPScale1Text = pianta.addText(str(scaleStep), QFont("Arial", 1))
            tmpPScale1Text.setPos(QPointF(pBounding[0]-pExtramargin+scaleStep+labelMargin, pBounding[1]-pExtramargin+scaleMargin+labelMargin))
            ScaleSXZ = QPainterPath()
            ScaleSXZ.moveTo(SXZscalePoint)
            SXZscalePoint = QPointF(sxzBounding[0]-pExtramargin+scaleStep, sxzBounding[1]-pExtramargin+scaleMargin)
            SXZscalePointDown = QPointF(sxzBounding[0]-pExtramargin+scaleStep, sxzBounding[1]-pExtramargin+scaleMargin+scaleWidth)
            ScaleSXZ.lineTo(SXZscalePoint)
            ScaleSXZ.lineTo(SXZscalePointDown)
            spaccatoXZ.addPath(ScaleSXZ, QPen(Qt.black, 0.1))
            tmpSXZScale1Text = spaccatoXZ.addText(str(scaleStep), QFont("Arial", 1))
            tmpSXZScale1Text.setPos(QPointF(sxzBounding[0]-pExtramargin+scaleStep+labelMargin, sxzBounding[1]-pExtramargin+scaleMargin+labelMargin))
            ScaleSYZ = QPainterPath()
            ScaleSYZ.moveTo(SYZscalePoint)
            SYZscalePoint = QPointF(syzBounding[0]-pExtramargin+scaleStep, syzBounding[1]-pExtramargin+scaleMargin)
            SYZscalePointDown = QPointF(syzBounding[0]-pExtramargin+scaleStep, syzBounding[1]-pExtramargin+scaleMargin+scaleWidth)
            ScaleSYZ.lineTo(SYZscalePoint)
            ScaleSYZ.lineTo(SYZscalePointDown)
            spaccatoYZ.addPath(ScaleSYZ, QPen(Qt.black, 0.1))
            tmpSYZScale1Text = spaccatoYZ.addText(str(scaleStep), QFont("Arial", 1))
            tmpSYZScale1Text.setPos(QPointF(syzBounding[0]-pExtramargin+scaleStep+labelMargin, syzBounding[1]-pExtramargin+scaleMargin+labelMargin))
        #Calculate data
        BarMin = 10000.0
        BarMax = -10000.0
        for row in Cfile["measurements"]:
            try:
                lunghezza = lunghezza + row["topographic"]["distance"]
                for b in range(len(self.branches)):
                    if row["to"] in self.branches[b]:
                        try:
                            oldbranchl = lunghezzarami[str(b)]
                        except:
                            oldbranchl = 0.0
                        lunghezzarami[str(b)] = oldbranchl + row["topographic"]["distance"]
            except:
                pass
            try:
                if row["ambient"]["bar_altitude"] == -10000:
                    continue
                if row["ambient"]["bar_altitude"] < BarMin:
                    BarMin = row["ambient"]["bar_altitude"]
                if row["ambient"]["bar_altitude"] > BarMax:
                    BarMax = row["ambient"]["bar_altitude"]
            except:
                pass
        try:
            dislivelloBar = BarMax - BarMin
        except:
            pass
        try:
            self.w.statusbar.showMessage("Lunghezza poligonale: "+str("{:.2f}".format(lunghezza))+" Dislivello: "+str("{:.2f}".format(dislivello))+" Dislivello barometrico: "+str("{:.2f}".format(dislivelloBar)))
            print("Lunghezza rami:")
            print(lunghezzarami)
            sStatText = "Polyline length (with pits depth): "+str("{:.2f}".format(lunghezza))+" \nHeight difference: "+str("{:.2f}".format(dislivello))+" \nBarometric height difference: "+str("{:.2f}".format(dislivelloBar))
            tmpSXZScale1Text = spaccatoXZ.addText(str(sStatText), QFont("Arial", 1))
            tmpSXZScale1Text.setPos(QPointF(sxzBounding[0]-pExtramargin+labelMargin, sxzBounding[1]-pExtramargin+scaleMargin+5))
            tmpSYZScale1Text = spaccatoYZ.addText(str(sStatText), QFont("Arial", 1))
            tmpSYZScale1Text.setPos(QPointF(syzBounding[0]-pExtramargin+labelMargin, syzBounding[1]-pExtramargin+scaleMargin+5))
        except:
            pass
        brtextspace = 1
        brtexty = pBounding[1]-pExtramargin+scaleMargin+5
        for b in range(len(self.branches)):
            try:
                branchtext = "Branch start: "+str(self.branches[b][0]) + " Branch end: "+str(self.branches[b][-1]) + " Branch length: "+str("{:.2f}".format(lunghezzarami[str(b)]))
                tmpPBrText = pianta.addText(str(branchtext), QFont("Arial", 1))
                tmpPBrText.setPos(QPointF(pBounding[0]-pExtramargin+labelMargin, brtexty))
                brtexty = brtexty + brtextspace
                print(branchtext)
            except:
                pass
        #show in graphicsview
        self.w.pianta.setScene(pianta)
        #TODO: aggiungere opzione in gui per scegliere XZ o YZ
        if self.w.spaccatocombo.currentText() == "YZ":
            self.w.spaccato.setScene(spaccatoYZ)
        else:
            self.w.spaccato.setScene(spaccatoXZ)
        self.w.pianta.show()
        self.w.spaccato.show()
        self.w.zoom.setValue(self.w.zoom.maximum())
        #files
        cleanedname = self.cleanName(self.w.cavename.currentText())
        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        Pfilename = cavefolder + "/" + cleanedname + "-pianta.svg"
        SYZfilename = cavefolder + "/" + cleanedname + "-spaccatoYZ.svg"
        SXZfilename = cavefolder + "/" + cleanedname + "-spaccatoXZ.svg"
        self.saveSvg(pianta, Pfilename, "Pianta")
        self.saveSvg(spaccatoYZ, SYZfilename, "Spaccato")
        self.saveSvg(spaccatoXZ, SXZfilename, "Spaccato")
        if self.dokml:
            kmlDoc.save(cavefolder + "/" + cleanedname + ".kml")

    def rotateNorth(self, point):
        #in our coordinates system, north goes from left to right. We need to change it from down to up: rotate 90° counter-clockwise
        x = point[0]
        y = point[1]
        radians = math.radians(90)
        x2 = x * math.cos(radians) + y * math.sin(radians)
        y2 = -x * math.sin(radians) + y * math.cos(radians)
        rotPoint = (x2,y2)
        return rotPoint


    #Ref.: https://gist.github.com/twpayne/4409500
    def UTMzone(self, coordinates):
        if 56 <= coordinates[1] < 64 and 3 <= coordinates[0] < 12:
            return 32
        if 72 <= coordinates[1] < 84 and 0 <= coordinates[0] < 42:
            if coordinates[0] < 9:
                return 31
            elif coordinates[0] < 21:
                return 33
            elif coordinates[0] < 33:
                return 35
            return 37
        return int((coordinates[0] + 180) / 6) + 1
    
    
    def UTMletter(self, coordinates):
        return 'CDEFGHJKLMNPQRSTUVWXX'[int((coordinates[1] + 80) / 8)]
    
    
    def UTMproject(self, coordinates):
        import pyproj
        z = self.UTMzone(coordinates)
        l = self.UTMletter(coordinates)
        if z not in self._projections:
            self._projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
        x, y = self._projections[z](coordinates[0], coordinates[1])
        if y < 0:
            y += 10000000
        return z, l, x, y
    
    
    def UTMunproject(self, z, l, x, y):
        import pyproj
        if z not in self._projections:
            self._projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
        if l < 'N':
            y -= 10000000
        lng, lat = self._projections[z](x, y, inverse=True)
        return (lng, lat)

    def drawSection(self, mysection = None, mysideTilt = None):
        if mysection == None:
            section = self.section
        else:
            section = mysection
        if mysideTilt == None:
            try:
                sideTilt = self.requiredData["sideTilt"]
            except:
                sideTilt = 0
        else:
            sideTilt = mysideTilt
        #print(mysection)
        #myCenter = [0.0, 0.0, 0.0]
        sezione = QGraphicsScene()
        #secCoords = self.calculateSectionCoord(section, myCenter, 0.0, 0.0, sideTilt)
        secCoords = []
        for i in range(len(section)):
            angle = (i - sideTilt) % len(section)
            myX = -section[i]*math.cos(math.radians(angle))
            myY = section[i]*math.sin(math.radians(angle))
            secCoords.append([myX,myY])
        if len(secCoords) <360 or self.isSectionNull(section):
            return sezione
        PennaBordo = QPen(Qt.black, 0.2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin) #Qt.Dashline https://doc.qt.io/qtforpython/PySide2/QtGui/QPen.html
        Spoligon = QPainterPath()
        Spoligon.moveTo(QPointF(secCoords[0][0], secCoords[0][1]))
        for angle in range(1,len(secCoords)):
            c1 = Spoligon.currentPosition()
            c2 = QPointF(secCoords[angle][0], secCoords[angle][1])
            Spoligon.cubicTo(c1, c2, QPointF(secCoords[angle][0], secCoords[angle][1]))
        c1 = Spoligon.currentPosition()
        c2 = QPointF(secCoords[0][0], secCoords[0][1])
        Spoligon.cubicTo(c1, c2, QPointF(secCoords[0][0], secCoords[0][1]))
        sezione.addPath(Spoligon, PennaBordo)
        #self.sezioneScene = sezione
        if mysection == None:
            self.sezioneScene = sezione
            self.w.section.setScene(self.sezioneScene)
            self.w.section.show()
            #QApplication.processEvents()
            #print(section)
        else:
            return sezione

    def isSectionNull(self, mysection):
        val = True
        for d in mysection:
            if d > 0.0:
                val = False
                break
        return val

    def saveSvg(self, scene, fileName, title = ""):
        generator = QSvgGenerator()
        generator.setFileName( fileName )
        generator.setSize( QSize( scene.width(), scene.height() ) )
        generator.setTitle(title);
        generator.setDescription("Created with Charlotte Cave Surveing Software");
        painter = QPainter( generator )
        #transform = QTransform().scale(1, -1) #we need a qtransform because in a qgraphicscene y-axis is flipped
        #transform.translate(0, -scene.height())
        #painter.setTransform(transform)
        scene.render( painter )
        painter.end()

    def zoomDrawings(self, val):
        zoom = self.w.zoom.value()/10
        #self.Json2Svg(self.myCaveFile)
        self.w.pianta.scale(zoom/self.currentzoom,zoom/self.currentzoom)
        self.w.spaccato.scale(zoom/self.currentzoom,zoom/self.currentzoom)
        self.currentzoom = zoom

    def Json2Dxf(self, Cfile):
        #Note: it's also possible to load this with Blender https://www.blender3darchitect.com/cad/importing-dxf-files-to-blender-2-8/
        print("Drawing DXF")
        #https://ezdxf.readthedocs.io/en/stable/tutorials/simple_drawings.html
        doc = ezdxf.new('R2010')  # create a new DXF R2010 drawing, official DXF version name: 'AC1024'

        msp = doc.modelspace()  # add new entities to the modelspace
        if self.w.dxfmesh.isChecked():
            mesh = msp.add_mesh()
        for row in Cfile["measurements"]:
            fromX = self.myCoordinates[row["from"]]["pos"][0]
            fromY = self.myCoordinates[row["from"]]["pos"][1]
            fromZ = self.myCoordinates[row["from"]]["pos"][2]
            toX = self.myCoordinates[row["to"]]["pos"][0]
            toY = self.myCoordinates[row["to"]]["pos"][1]
            toZ = self.myCoordinates[row["to"]]["pos"][2]
            #print((fromX, fromY, fromZ), (toX, toY, toZ))
            thickness = 0
            #https://ezdxf.readthedocs.io/en/stable/layouts/layouts.html#ezdxf.layouts.BaseLayout.add_line
            msp.add_line((fromX, -fromY, fromZ), (toX, -toY, toZ))  # add a LINE entity
            myCenter = [fromX, fromY, fromZ]
            if self.isValidSection(row["section"]):
                secCoords = self.calculateSectionCoord(row["section"], myCenter, row["topographic"]["heading"], row["topographic"]["frontalInclination"], row["topographic"]["sideTilt"])
            else:
                l = row["walls"]['left']
                r = row["walls"]['right']
                u = row["walls"]['up']
                d = row["walls"]['down']
                section_from_walls = self.sectionFromWalls(l,r,u,d)
                secCoords = self.calculateSectionCoord(section_from_walls, myCenter, row["topographic"]["heading"], row["topographic"]["frontalInclination"], row["topographic"]["sideTilt"])
                #print(section_from_walls)
            if len(secCoords)>0:
                for secP in range(len(secCoords)):
                    sFx = secCoords[secP][0]
                    sFy = secCoords[secP][1]
                    sFz = secCoords[secP][2]
                    if secP == (len(secCoords)-1):
                        sTx = secCoords[0][0]
                        sTy = secCoords[0][1]
                        sTz = secCoords[0][2]
                    else:
                        sTx = secCoords[secP+1][0]
                        sTy = secCoords[secP+1][1]
                        sTz = secCoords[secP+1][2]
                    msp.add_line((sFx, -sFy, sFz), (sTx, -sTy, sTz))
            if self.w.dxfmesh.isChecked() and len(secCoords)>0:
                try:
                    toRow = self.getFromPointData(Cfile, row["to"])
                    if self.isValidSection(toRow["section"]):
                        ToSection = toRow["section"]
                    else:
                        l = toRow["walls"]['left']
                        r = toRow["walls"]['right']
                        u = toRow["walls"]['up']
                        d = toRow["walls"]['down']
                        ToSection = self.sectionFromWalls(l,r,u,d)
                    myCenter = [toX,toY,toZ]
                    ToSecCoords = self.calculateSectionCoord(ToSection, myCenter, toRow["topographic"]["heading"], toRow["topographic"]["frontalInclination"], toRow["topographic"]["sideTilt"])
                    endofbranch = False
                except:
                    endofbranch = True
                for angle in range(360):
                    i = angle%len(secCoords)
                    n = (angle+1)%len(secCoords)
                    aX = secCoords[i][0]
                    aY = secCoords[i][1]
                    aZ = secCoords[i][2]
                    bX = secCoords[n][0]
                    bY = secCoords[n][1]
                    bZ = secCoords[n][2]
                    try:
                        if not endofbranch:
                            cX = ToSecCoords[i][0]
                            cY = ToSecCoords[i][1]
                            cZ = ToSecCoords[i][2]
                            dX = ToSecCoords[n][0]
                            dY = ToSecCoords[n][1]
                            dZ = ToSecCoords[n][2]
                    except:
                        endofbranch = True
                    if not endofbranch:
                        #https://ezdxf.readthedocs.io/en/stable/tutorials/mesh.html
                        with mesh.edit_data() as mesh_data:
                            mesh_data.add_face([(aX,-aY,aZ), (cX,-cY,cZ), (dX,-dY,dZ), (bX,-bY,bZ)])
                    else:
                        with mesh.edit_data() as mesh_data:
                            mesh_data.add_face([(aX,-aY,aZ), (toX,-toY,toZ), (bX,-bY,bZ)])
                mesh_data.optimize()  # optional, minimizes vertex count
        cleanedname = self.cleanName(self.w.cavename.currentText())
        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        nomesh = ""
        if not self.w.dxfmesh.isChecked():
            nomesh = "-nomesh"
        Dfilename = cavefolder + "/" + cleanedname + nomesh + ".dxf"
        doc.saveas(Dfilename)

    def sectionFromWalls(self, l, r, u, d, tilt = 0):
        section_from_walls = []
        for alpha in range(0,360):
            if (alpha+tilt) == 0:
                dist = l
            elif (alpha+tilt) == 90:
                dist = d
            elif (alpha+tilt) == 180:
                dist = r
            elif (alpha+tilt) == 270:
                dist = u
            else:
                Ax=0
                Ay=0
                Bx=0
                By=0
                if (alpha+tilt) > 0 and (alpha+tilt) < 90:
                    Ax=l
                    Ay=0
                    Bx=0
                    By=-d
                if (alpha+tilt) > 90 and (alpha+tilt) < 180:
                    Ax=0
                    Ay=-d
                    Bx=-r
                    By=0
                if (alpha+tilt) > 180 and (alpha+tilt) < 270:
                    Ax=-r
                    Ay=0
                    Bx=0
                    By=u
                if (alpha+tilt) > 270 and (alpha+tilt) < 360:
                    Ax=0
                    Ay=u
                    Bx=l
                    By=0
                try:
                    dist = ((Ax*By-Ay*Bx)/((Bx-Ax)*math.sin(math.radians(180-alpha))+(Ay-By)*math.cos(math.radians(180-alpha))))
                except:
                    dist = 0
            section_from_walls.append(dist)
        return section_from_walls

    def isValidSection(self, section):
        for dist in section:
            if dist != 0.0:
                return True
        return False

    def updatedrawing(self):
        self.getCoordinates(self.myCaveFile)
        self.getBranches(self.myCaveFile)
        self.Json2Svg(self.myCaveFile)
        #xfiletxt = self.json2CSX(self.myCaveFile)
        #print(xfiletxt)
        
    def updatedxf(self):
        self.getCoordinates(self.myCaveFile)
        self.getBranches(self.myCaveFile)
        self.Json2Dxf(self.myCaveFile)
        
    def spaccatocombo(self):
        pass
    
    def piantacombo(self):
        pass
    
    def updatelidarsvg(self):
        cleanedname = self.cleanName(self.w.cavename.currentText())
        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        secfolder = cavefolder + "/" + "sections"
        try:
            os.makedirs(secfolder)
        except:
            pass
        processed = []
        for row in self.myCaveFile["measurements"]:
            pname = str(row["from"])
            processed.append(pname)
            if processed.count(pname) == 1:
                pfilename = pname
            else:
                pfilename = pname+"-"+str(processed.count(pname))
            mysection = row["section"]
            mysideTilt = row["topographic"]["sideTilt"]
            secScene = self.drawSection(mysection, mysideTilt)
            Sfilename = secfolder + "/" + cleanedname + "-sezione"+pfilename+".svg"
            self.saveSvg(secScene, Sfilename, "Sezione "+pfilename)        

    def getFromPointData(self, Cfile, pointname, topointname = ""):
        point = {}
        for row in Cfile["measurements"]:
            if row["from"] == pointname:
                point = row
                if topointname == "" or topointname == row["to"]:
                    break
        return point

    def getBranches(self, Cfile):
        rami = []
        #looking = True
        startpoint = self.checkPointNotBranched(rami)
        r = 0
        while startpoint != "":
            if r >= len(rami):
                rami.append([])
            if len(rami[r]) == 0:   #just started a new branch, look for its first from point (if exists)
                for row in Cfile["measurements"]:
                    Tpoint = row["to"]
                    Fpoint = row["from"]
                    if Tpoint==startpoint and not self.checkPointBranched(rami, Tpoint):
                        rami[r].append(Fpoint)
            found = False
            for row in Cfile["measurements"]:
                Tpoint = row["to"]
                Fpoint = row["from"]
                if Fpoint==startpoint and not self.checkPointBranched(rami, Fpoint):
                    rami[r].append(startpoint)
                    startpoint = Tpoint
                    found = True
                    break
            if not found:
                #if this point is a "to" but not a "from", means we reached the end of this branch
                rami[r].append(startpoint)
                r = r +1
                #go on until there is a point still not considered
                startpoint = self.checkPointNotBranched(rami)
        self.branches = rami
        print("Total number of branches: "+str(len(self.branches)))

    def checkPointNotBranched(self, rami):
        looking = True
        startpoint = ""
        for key in self.myCoordinates:
            looking = True
            for el in rami:
                for pname in el:
                    if pname == key:
                        looking = False
                        break
                if looking == False:
                    break
            if looking == True:
                startpoint = key
                break
        return startpoint

    def checkPointBranched(self, rami, pointname):
        found = False
        for el in rami:
            for pname in el:
                if pname == pointname:
                    found = True
                    break
            if found == True:
                break
        return found

    def deleterow(self):
        self.deletefromtable(False)

    def deletefixrow(self):
        self.deletefromtable(True)
    
    def deletefromtable(self, autofix = False):
        selIndex = self.w.fulltable.selectedItems()[-1].row() #vale solo l'ultimo selezionato
        thisFrom = self.myCaveFile['measurements'][selIndex]['from']
        thisTo = self.myCaveFile['measurements'][selIndex]['to']
        print(self.myCaveFile['measurements'][selIndex])
        del self.myCaveFile['measurements'][selIndex]
        if autofix:
            for i in range(len(self.myCaveFile['measurements'])):
                if self.myCaveFile['measurements'][i]['to'] == thisFrom:
                   self.myCaveFile['measurements'][i]['to'] = thisTo
        self.saveFile()
        
    def savetable(self, autofix = False):
        selIndex = self.w.fulltable.selectedItems()[-1].row() #vale solo l'ultimo selezionato
        for r in range(self.w.fulltable.rowCount()):
            #self.csvheader = ["From", "To", "SideTilt", "FrontalInclination", "heading", "distance", "left", "right", "up", "down"]
            self.myCaveFile['measurements'][r]['from'] = self.w.fulltable.item(r,0).text()
            self.myCaveFile['measurements'][r]['to'] = self.w.fulltable.item(r,1).text()
            self.myCaveFile['measurements'][r]['topographic']['sideTilt'] = float(self.w.fulltable.item(r,2).text())
            self.myCaveFile['measurements'][r]['topographic']['frontalInclination'] = float(self.w.fulltable.item(r,3).text())
            self.myCaveFile['measurements'][r]['topographic']['heading'] = float(self.w.fulltable.item(r,4).text())
            self.myCaveFile['measurements'][r]['topographic']['distance'] = float(self.w.fulltable.item(r,5).text())
            self.myCaveFile['measurements'][r]['walls']['left'] = float(self.w.fulltable.item(r,6).text())
            self.myCaveFile['measurements'][r]['walls']['right'] = float(self.w.fulltable.item(r,7).text())
            self.myCaveFile['measurements'][r]['walls']['up'] = float(self.w.fulltable.item(r,8).text())
            self.myCaveFile['measurements'][r]['walls']['down'] = float(self.w.fulltable.item(r,9).text())
            self.myCaveFile['measurements'][r]['GPS']['latitude'] = float(self.w.fulltable.item(r,10).text())
            self.myCaveFile['measurements'][r]['GPS']['longitude'] = float(self.w.fulltable.item(r,11).text())
            self.myCaveFile['measurements'][r]['GPS']['altitude'] = float(self.w.fulltable.item(r,12).text())
            self.myCaveFile['measurements'][r]['notes'] = str(self.w.fulltable.item(r,13).text())
        self.saveFile()

    def populateTable(self, CSV):
        #TODO: delete rows and columns
        self.w.fulltable.setRowCount(0)
        for i in range(self.w.fulltable.columnCount()):
            self.w.fulltable.removeColumn(0)
        if CSV == "":
            return
        r = 0
        for row in CSV.split("\n"):
            if len(row.split(","))<2:
                continue
            c = 0
            self.addlinetotable("", c)
            for col in row.split(","):
                if r==0:
                    self.addcolumn(self.csvheader[c], c)
                self.setcelltocorpus(col, r, c)
                c = c + 1
            r = r + 1

    def addcolumn(self, text, column):
        cols = self.w.fulltable.columnCount()
        self.w.fulltable.setColumnCount(cols+1)
        titem = QTableWidgetItem()
        titem.setText(text)
        self.w.fulltable.setHorizontalHeaderItem(cols, titem)

    def addlinetotable(self, text, column):
        #self.w.fulltable.cellChanged.disconnect(self.corpusCellChanged)
        row = self.w.fulltable.rowCount()
        self.w.fulltable.insertRow(row)
        titem = QTableWidgetItem()
        titem.setText(text)
        self.w.fulltable.setItem(row, column, titem)
        self.w.fulltable.setCurrentCell(row, column)
        #self.w.fulltable.cellChanged.connect(self.corpusCellChanged)
        return row

    def setcelltocorpus(self, text, row, column):
        #self.w.fulltable.cellChanged.disconnect(self.corpusCellChanged)
        titem = QTableWidgetItem()
        titem.setText(text)
        self.w.fulltable.setItem(row, column, titem)
        #self.w.fulltable.cellChanged.connect(self.corpusCellChanged)

    def cavenamechanged(self):
        cleanedname = self.cleanName(self.w.cavename.currentText())
        self.mycfg["lastcave"] = cleanedname
        #self.savePersonalCFG()

    def openFile(self):
        cleanedname = self.mycfg["lastcave"]
        Cfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".json"
        Cfile = self.openJson(Cfilename)
        #self.savePersonalCFG()
        self.w.fromP.setText("0")
        self.w.toP.setText("1")
        self.populateTable("")
        if len(Cfile)>0:
            print(Cfile['caveName'])
            if self.w.cavename.findText(Cfile['caveName'])<0:
                self.w.cavename.addItem(Cfile['caveName'])
            self.w.cavename.setCurrentText(Cfile['caveName'])
            if len(Cfile['measurements'])>0:
                self.w.fromP.setText(Cfile['measurements'][-1]['from'])
                self.w.toP.setText(Cfile['measurements'][-1]['to'])
                self.incrementFromTo()
                tfiletxt = self.json2CSV(Cfile)
                self.populateTable(tfiletxt)
        self.myCaveFile = Cfile

    def openJson(self, fileName):
        folder = os.path.abspath(os.path.dirname(fileName))
        if not os.path.isdir(folder):
            os.makedirs(folder)
        if not os.path.isfile(fileName):
            text_file = open(fileName, "w", encoding='utf-8')
            text_file.write("")
            text_file.close()
        try:
            text_file = open(fileName, "r", encoding='utf-8')
            lines = text_file.read()
            text_file.close()
            out = json.loads(lines.replace("\n", "").replace("\r", ""))
            return out
        except:
            return {}

    def listCaves(self):
        self.w.cavename.clear()
        for cleanedname in os.listdir(self.mycfg["outputfolder"]):
            Cfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".json"
            if os.path.isfile(Cfilename):
                self.w.cavename.addItem(cleanedname)
        try:
            if self.w.cavename.findText(self.myCaveFile['caveName'])<0:
                self.w.cavename.addItem(self.myCaveFile['caveName'])
            self.w.cavename.setCurrentText(self.myCaveFile['caveName'])
        except:
            pass

    def SaveSurvey(self):
        if self.w.save.isChecked():
            self.w.save.setText("Sto salvando...")
            #self.saveFile()
            self.appendNewPoint()
            self.w.save.setChecked(False)
            self.w.save.setText("Salva misurazione")
            if self.mycfg["lastcave"] == "":
                return
            #increment from and to
            self.incrementFromTo()
        else:
            self.w.save.setText("Salva misurazione")

    def puntofisso(self):
        #In questo caso catturiamo i dati ogni 2 secondi, e sottraiamo la distanza dal primo punto
        self.firstdistance = float(self.w.distance.value())

    def puntofissoSave(self):
        active = True
        firstdistance = float(self.w.distance.value())
        stopped = False
        toWait = 5 #self.w.autoscanSleep.value()
        toSleep = 0.1
        firstrun = True
        while active:
            if self.w.puntofisso.isChecked():
                stopped = False
                newdistance = float(self.w.distance.value())
                if firstrun:
                    firstdistance = newdistance
                    firstrun = False
                    self.w.puntofisso.setStyleSheet("background-color: rgb(127, 127, 127);")
                    time.sleep(toWait/2)
                    continue
                if (firstdistance-newdistance)<=0:
                    self.w.puntofisso.setStyleSheet("background-color: rgb(255, 0, 0);")
                    time.sleep(toWait/2)
                    continue
                else:
                    self.w.puntofisso.setStyleSheet("background-color: rgb(0, 255, 0);")
                if self.mycfg["lastcave"] == "":
                    print("Error: lastcave is null")
                    return
                self.appendNewPoint(firstdistance, doDraw=False)
                #increment from and to
                self.incrementFromTo()
                firstdistance = newdistance
                firstrun = False
                self.w.puntofisso.setStyleSheet("background-color: rgb(127, 127, 127);")
                QApplication.processEvents()
            startTS = datetime.now().timestamp()
            print("Wait "+str(toWait)+" seconds")
            doWait = True
            while doWait:
                time.sleep(toSleep)
                if self.w.puntofisso.isChecked():
                    doWait = bool((datetime.now().timestamp()-startTS) < toWait)
                elif not stopped:
                    stopped = True
                    print("Stopping autosave timer")
                    self.w.puntofisso.setStyleSheet("")
                    firstrun = True
                    time.sleep(toSleep)
                    #QApplication.processEvents()
                    time.sleep(toSleep)
                else:
                    doWait = True
                if stopped and self.w.puntofisso.isChecked():
                    break
        return None

    def incrementFromTo(self):
        self.w.fromP.setText(self.w.toP.text())
        newto = self.w.toP.text()
        for i in range(len(newto)):
            try:
                newto = newto[0:i] + str(int(newto[i:])+1)
                break
            except:
                continue
        if newto == self.w.toP.text():
            newto = newto+"1"
        self.w.toP.setText(newto)

    def calibrateTilt(self):
        #https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration
        print("offset: ")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.setFixedSize(640,480)
    w.show()
    sys.exit(app.exec_())
