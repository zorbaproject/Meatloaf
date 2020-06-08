#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, os
from time import sleep
import json
import math
import re
from datetime import datetime


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
from PySide2.QtWidgets import QTableWidget
from PySide2.QtWidgets import QTableWidgetItem
from PySide2.QtWidgets import QTableWidgetSelectionRange

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
        while True:
            if self.myparent.w.manualMode.isChecked():
                sleep(toSleep)
                continue
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
        self.w.save.clicked.connect(self.SaveSurvey)
        self.w.openlastcave.stateChanged.connect(self.EditConf)
        self.w.exit.clicked.connect(self.chiudi)
        self.w.shutdown.clicked.connect(self.shutdown)
        self.w.reboot.clicked.connect(self.reboot)
        self.w.accesspoint.clicked.connect(self.accesspoint)
        #self.w.tempImpostata.valueChanged.connect(self.setTempImp)
        self.w.cavename.textChanged.connect(self.cavenamechanged)
        self.requiredData = {}
        self.section = []
        self.walls = {}
        self.mycfgfile = QDir.homePath() + "/.charlottecfg"
        self.mycfg = {}
        self.newCave()
        self.csvheader = ["From", "To", "SideTilt", "FrontalInclination", "heading", "distance", "left", "right", "up", "down"]
        self.loadPersonalCFG()
        if os.path.isfile(self.mycfg['lastcave']) and self.mycfg['startfromlastcave']=='True':
            self.openFile()
        if isRPI:
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

    def accesspoint(self):
        if isRPI:
            print("Access Point mode")
            os.system(os.path.abspath(os.path.dirname(sys.argv[0]))+'/accesspoint.sh')
            self.w.statusbar.showMessage("Please connect to charlotte.lan or 192.168.1.1")
            #attiviamo la scheda wifi
            #abilitiamo un samba share sulla cartella dei rilievi
            #abilitiamo un server web con listing dei file sulla cartella dei rilievi
            #diciamo all'utente di connettersi a 192.168.1.1

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
        }
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
        if isRPI and not self.w.manualMode.isChecked():
            #self.myThread = TurnOn(self.w)
            #self.myThread.TempReached.connect(self.reached)
            #self.myThread.finished.connect(self.itIsOff)
            #self.myThread.start()
            #self.alreadyOn = True
            print("Reading data from lidar scanner")
        if len(output) < 360:
            output = [0.0 for deg in range(360)]  #we expect to get one value for every degree
        else:
            self.calculateWalls(output)
        self.section = output
        #print(output)
        #self.calculateWalls(output)

    def calculateWalls(self, section):
        leftWall = self.averageListRange(section, 45,135)
        upWall = self.averageListRange(section, 135,225)
        rightWall = self.averageListRange(section, 225,315)
        downWall = (self.averageListRange(section, 315,360)+self.averageListRange(section, 0,45))/2
        self.walls = {'left':leftWall, 'right':rightWall, 'up':upWall, 'down':downWall}
        self.w.leftW.setValue(leftWall)
        self.w.rightW.setValue(rightWall)
        self.w.upW.setValue(upWall)
        self.w.downW.setValue(downWall)

    def averageListRange(self, mylist, myfrom = 0, myto = None):
        if myto == None:
            myto = len(mylist)
        if myfrom >= len(mylist):
            myfrom = len(mylist)-1
        if myto > len(mylist):
            myto = len(mylist)
        sum = 0
        for el in range(myfrom, myto):
            try:
                sum = sum + float(el)
            except:
                continue
        average = sum/(myto-myfrom)
        return average

    def cleanName(self, name):
        cleanedname = re.sub("[^0-9A-Za-z\_\-]", "_", name)
        return cleanedname

    def saveFile(self):
        cleanedname = self.cleanName(self.w.cavename.text())
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
        print("Saving to " + Cfilename)

        temp = self.w.temperature.value()
        press = self.w.pressure.value()

        requiredData = {
        'sideTilt':self.w.sideTilt.value(),
        'frontalInclination':self.w.frontalInclination.value(),
        'heading':self.w.heading.value(),
        'distance':self.w.distance.value()
        }

        walls = {'left':self.w.leftW.value(), 'right':self.w.rightW.value(), 'up':self.w.upW.value(), 'down':self.w.downW.value()}

        Cfiletemplate = {'caveName':self.w.cavename.text(), 'measurements':[]}
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
        'topographic': requiredData,
        'walls': walls,
        'section': self.section,
        'ambient':{'temperature': temp, 'pressure': press}
        }

        Cfile['measurements'].append(thismeasure)
        cfiletxt = json.dumps(Cfile)
        text_file = open(Cfilename, "w", encoding='utf-8')
        text_file.write(cfiletxt)
        text_file.close()

        #now we build the CSV based on the measurements
        tfiletxt = self.json2CSV(Cfile)
        text_file = open(Tfilename, "w", encoding='utf-8')
        text_file.write(tfiletxt)
        text_file.close()
        self.populateTable(tfiletxt)

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

            #csvtxt = csvtxt + str(row["ambient"]['temperature']) + ","
            #csvtxt = csvtxt + str(row["ambient"]['pressure']) + ","

            #for num in row["section"]:
            #   csvtxt = csvtxt + str(num) + ";"
            #csvtxt = csvtxt + ","

            csvtxt = csvtxt[0:-1] + "\n"
        return csvtxt

    def populateTable(self, CSV):
        #TODO: delete rows and columns
        self.w.fulltable.setRowCount(0)
        for i in range(self.w.fulltable.columnCount()):
            self.w.fulltable.removeColumn(0)
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
        cleanedname = self.cleanName(self.w.cavename.text())
        self.mycfg["lastcave"] = cleanedname
        #self.savePersonalCFG()

    def openFile(self):
        cleanedname = self.mycfg["lastcave"]
        Cfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".json"
        Cfile = self.openJson(Cfilename)
        #self.savePersonalCFG()
        if len(Cfile)>0:
            self.w.cavename.setText(Cfile['caveName'])
            if len(Cfile['measurements'])>0:
                self.w.fromP.setText(Cfile['measurements'][-1]['from'])
                self.w.toP.setText(Cfile['measurements'][-1]['to'])
                self.incrementFromTo()
                tfiletxt = self.json2CSV(Cfile)
                self.populateTable(tfiletxt)

    def openJson(self, fileName):
        try:
            text_file = open(fileName, "r", encoding='utf-8')
            lines = text_file.read()
            text_file.close()
            out = json.loads(lines.replace("\n", "").replace("\r", ""))
            return out
        except:
            return {}

    def SaveSurvey(self):
        if self.w.save.isChecked():
            self.w.save.setText("Sto salvando...")
            self.doLidarScan()
            self.saveFile()
            self.w.save.setChecked(False)
            self.w.save.setText("Salva misurazione")
            if self.mycfg["lastcave"] == "":
                return
            #increment from and to
            self.incrementFromTo()
        else:
            self.w.save.setText("Salva misurazione")

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
