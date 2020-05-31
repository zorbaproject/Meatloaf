#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, os
from time import sleep
import json
import math


try:
    #Devono essere abilitati i 1-Wire
    os.system('sudo modprobe w1-gpio')
    os.system('sudo modprobe w1-therm')

    import RPi.GPIO as GPIO
    import w1thermsensor
    import board
    import busio
    import adafruit_lsm303_accel
    import adafruit_lsm303dlh_mag
    isRPI = True
except:
    isRPI = False


try:
    from PySide2.QtWidgets import QApplication
except:
    try:
        from tkinter import messagebox
        messagebox.showinfo("Installazione, attendi prego", "Sto per installare le librerie grafiche e ci vorrà del tempo. Premi Ok e vai a prenderti un caffè.")
        pip.main(["install", "PySide2"])
        from PySide2.QtWidgets import QApplication
    except:
        try:
            from pip._internal import main
            main(["install", "PySide2"])
            from PySide2.QtWidgets import QApplication
        except:
            sys.exit(1)


from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtCore import QDir
from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtCore import QThread


toSleep = 1
samples = 5 #number of samples to take for calculating average

class getLidar(QThread):
    TempReached = Signal(bool)

    def __init__(self, w, addr = ""):
        QThread.__init__(self)
        self.w = w
        self.setTerminationEnabled(True)
        self.relaypin = 23
        if not isRPI:
            self.exit()
        # Numerazione dei PIN GPIO
        GPIO.setmode(GPIO.BCM)
        # Il pin del relay va in output
        #GPIO.setup(self.relaypin, GPIO.OUT)
        #Cerco il sensore
        #self.sensor = w1thermsensor.W1ThermSensor()

    def __del__(self):
        print("Shutting down thread")

    def run(self):
        self.reachTemp()
        return

    def reachTemp(self):
        global toSleep
        try:
            while self.w.save.isChecked():
                if float(self.readTemp()) < float(self.w.tempImpostata.value()):
                    # Accendo il relay
                    GPIO.output(self.relaypin, GPIO.HIGH)
                    sleep(toSleep) 
                else:
                    # Spengo il relay
                    GPIO.output(self.relaypin, GPIO.LOW)
                    self.TempReached.emit(True)
        except:
            self.TempReached.emit(False)


class getData(QThread):
    #######################################Here we read temperature, pressure, distance, and 3-axis position
    def __init__(self, parent, mydata = ""):
        QThread.__init__(self)
        self.myparent = parent
        self.setTerminationEnabled(True)
        if not isRPI:
            self.exit()
        #Cerco i sensori
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
        self.accel = adafruit_lsm303_accel.LSM303_Accel(self.i2c)
        try:
            self.sensor = w1thermsensor.W1ThermSensor()
        except:
            self.sensor = None

    def __del__(self):
        print("Shutting down thread")

    def run(self):
        global toSleep
        while not self.myparent.w.manualMode.isChecked():
            self.requiredData = {}
            try:
                self.requiredData["temperature"] = self.readTemp()
            except:
                self.requiredData["temperature"] = -127
            mag_x, mag_y, mag_z = self.mag.magnetic
            accel_x,accel_y,accel_z = self.accel.acceleration
            self.requiredData["sideTilt"] = self.get_x_rotation(accel_x,accel_y,accel_z)
            self.requiredData["frontalInclination"] = self.get_y_rotation(accel_x,accel_y,accel_z)
            self.requiredData["heading"] = self.get_heading(mag_x,mag_y,mag_z)
            self.requiredData["distance"] = 0.0
            self.myparent.setRequiredData(self.requiredData)
            sleep(toSleep)
        return
    
    def readTemp(self):
        #Leggo la temperatura
        temperature_in_celsius = self.sensor.get_temperature()
        return temperature_in_celsius

    def dist(self, a,b):
        return math.sqrt((a*a)+(b*b))

    def get_y_rotation(self, x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return math.degrees(radians)

    def get_x_rotation(self, x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)


    def get_heading(self, x,y,z):
        if math.fabs(z) < 3:
            print("Tilting too much, heading measurement could be unaccurate")
        radians = math.atan2(y,x)
        heading = math.degrees(radians)
        if heading <0:
            heading = heading + 360
        return heading

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        file = QFile(os.path.abspath(os.path.dirname(sys.argv[0]))+"/charlotte.ui")
        file.open(QFile.ReadOnly)
        loader = QUiLoader(self)
        self.w = loader.load(file)
        self.setCentralWidget(self.w)
        self.setWindowTitle("Charlotte")
        self.w.save.clicked.connect(self.StopThis)
        self.w.exit.clicked.connect(self.chiudi)
        self.w.shutdown.clicked.connect(self.shutdown)
        self.w.reboot.clicked.connect(self.reboot)
        #self.w.tempImpostata.valueChanged.connect(self.setTempImp)
        self.requiredData = {}
        self.section = []
        self.mycfgfile = QDir.homePath() + "/.charlottecfg"
        self.mycfg = {}
        self.loadPersonalCFG()
        if isRPI:
            self.stoponreached = False
            self.getDataThread = getData(self)
            self.getDataThread.start()

    def chiudi(self):
        sys.exit(0)

    def shutdown(self):
        if isRPI:
            os.system('sudo halt')

    def reboot(self):
        if isRPI:
            os.system('sudo reboot')

    def loadPersonalCFG(self):
        try:
            text_file = open(self.mycfgfile, "r", encoding='utf-8')
            lines = text_file.read()
            text_file.close()
            self.mycfg = json.loads(lines.replace("\n", "").replace("\r", ""))
        except:
            print("Creo il file di configurazione")
        cfgtemplate = {'outputfolder': '', 'lastcave': '', 'startfromlastcave': '', 'sessions': [],
        'calibration': {
        'x': {'min': '', 'max':''},
        'y': {'min': '', 'max':''},
        'z': {'min': '', 'max':''},
        'distance': ''
        }
        }
        for key in cfgtemplate:
            if key not in self.mycfg:
                self.mycfg[key] = cfgtemplate[key]
                self.savePersonalCFG()

    def savePersonalCFG(self):
        cfgtxt = json.dumps(self.mycfg)
        text_file = open(self.mycfgfile, "w", encoding='utf-8')
        text_file.write(cfgtxt)
        text_file.close()

    def itIsOff(self):
        self.w.save.setChecked(False)
        self.StopThis()
        
    def reached(self):
        print("Temperatura raggiunta")
        if self.stoponreached:
            self.itIsOff()
        
    def setRequiredData(self, mydata):
        self.requiredData = mydata
        self.validateData()
        self.w.temperature.setValue(self.requiredData["temperature"])
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

    def doLidarScan(self):
        output = []
        if isRPI:
            #self.myThread = TurnOn(self.w)
            #self.myThread.TempReached.connect(self.reached)
            #self.myThread.finished.connect(self.itIsOff)
            #self.myThread.start()
            #self.alreadyOn = True
            print("Reading data from lidar scanner")
        if len(output) < 360:
            output = [0.0 for deg in range(360)]  #we expect to get one value for every degree

    def saveFile(self, fileName, requiredData, sectionData):
        print("Saving to " + fileName)
        print(requiredData)

    def openFile(self, fileName):
        self.requiredData

    def StopThis(self):
        if self.w.save.isChecked():
            self.w.save.setText("Sto salvando...")
            self.doLidarScan()
            self.w.save.setChecked(False)
        else:
            self.w.save.setText("Salva misurazione")


    def calibrateTilt(self):
        #https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration
        print("offset: ")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.setFixedSize(640,480)
    w.show()
    sys.exit(app.exec_())
