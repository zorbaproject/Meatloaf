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

import ezdxf

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
    import adafruit_lsm303_accel
    import adafruit_lsm303dlh_mag
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
from PySide2.QtSvg import QSvgGenerator
from PySide2.QtGui import QPainter
from PySide2.QtGui import QPainterPath
from PySide2.QtGui import QPolygonF
from PySide2.QtGui import QPen
from PySide2.QtGui import QColor
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF

import threading

print("All modules imported")

toSleep = 1
samples = 5 #number of samples to take for calculating average

class getLidar(QThread):
    global isRPI
    GotScan = Signal(list)

    def __init__(self, parent):
        QThread.__init__(self)
        self.myparent = parent
        self.setTerminationEnabled(True)
        if not isRPI:
            self.exit()
        # We just keep this in case we need to drive a servo
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(self.relaypin, GPIO.OUT)
        self.scantime = 2
        self.lidarport = self.findYDLidarX4(["/dev/ttyUSB0", "/dev/ttyUSB1"])
        print("Found Lidar on " + str(self.lidarport))
        self.lidarAddress = self.findUSBaddress("cp210x")
        # "dmesg | grep ': cp210x converter detected' |sed 's/\[.*\] cp210x \(.*\):.*/\1/g' | tail -n1"
        print("Lidar Address: "+self.lidarAddress)


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
            if self.myparent.w.manualMode.isChecked() or self.lidarport == None:
                #self.stopRangefinder()
                sleep(toSleep)
                continue
            myscan = self.scanYDLidarX4()
            #self.myparent.LidarScanDone(myscan)
            self.GotScan.emit(myscan)
            sleep(1)
            #self.drawSection()
        return

    def findYDLidarX4(self, ttys = ["/dev/ttyUSB0", "/dev/ttyUSB1"]):
        print("Searching for Lidar on ")
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

class getData(QThread):
    #######################################Here we read temperature, pressure, distance, and 3-axis position
    def __init__(self, parent, mydata = ""):
        QThread.__init__(self)
        self.myparent = parent
        self.setTerminationEnabled(True)
        if not isRPI:
            self.exit()
        #Data for the rangefinder
        self.rangefinderTTY = '/dev/ttyUSB0'
        self.RFbaudrate = 19200
        self.ledoffcommand = b'C'
        self.ledoncommand = b'O'
        self.distancecommand = b'D'
        self.ledoncode = "OK!"  #This is the response to look for after turning on led
        self.distancecode = "m," #This is the response to look for after requesting a distance measurement
        #Cerco i sensori
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
            self.accel = adafruit_lsm303_accel.LSM303_Accel(self.i2c)
        except:
            print("Unable to find LSM303DLH compass and inclinometer.")
            self.i2c = None
            self.mag = None
            self.accel = None
        self.rangefinderTTY = self.searchRangefinder(['/dev/ttyUSB0','/dev/ttyUSB1'])
        print("Rangefinder: " + str(self.rangefinderTTY))
        try:
            self.tempsensor = w1thermsensor.W1ThermSensor()
        except:
            self.tempsensor = None

    def __del__(self):
        print("Shutting down thread")

    def run(self):
        global toSleep
        while True:
            if self.myparent.w.manualMode.isChecked():
                self.stopRangefinder()
                sleep(toSleep)
                continue
            self.requiredData = {}
            try:
                self.requiredData["temperature"] = self.readTemp()
            except:
                self.requiredData["temperature"] = -127
            try:
                mag_x, mag_y, mag_z = self.mag.magnetic
                accel_x,accel_y,accel_z = self.accel.acceleration
                self.requiredData["sideTilt"] = self.get_x_rotation(accel_x,accel_y,accel_z)
                self.requiredData["frontalInclination"] = self.get_y_rotation(accel_x,accel_y,accel_z)
                self.requiredData["heading"] = self.get_heading(mag_x,mag_y,mag_z)
            except:
                self.requiredData["sideTilt"] = 0.0
                self.requiredData["frontalInclination"] = 0.0
                self.requiredData["heading"] = 0.0
            try:
                self.requiredData["distance"] = self.getDistance()
            except:
                self.requiredData["distance"] = 0.0
            self.myparent.setRequiredData(self.requiredData)
            sleep(toSleep)
        return
    
    def readTemp(self):
        #Leggo la temperatura
        temperature_in_celsius = self.tempsensor.get_temperature()
        return temperature_in_celsius

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
                        lookfordistance = False
            except:
                sleep(0.1)
        return dist

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
        print("Loading UI")
        file = QFile(os.path.abspath(os.path.dirname(sys.argv[0]))+"/charlotte.ui")
        file.open(QFile.ReadOnly)
        loader = QUiLoader(self)
        self.w = loader.load(file)
        self.setCentralWidget(self.w)
        self.setWindowTitle("Charlotte")
        self.w.save.clicked.connect(self.SaveSurvey)
        self.w.puntofisso.clicked.connect(self.puntofisso)
        self.w.openlastcave.stateChanged.connect(self.EditConf)
        self.w.exit.clicked.connect(self.chiudi)
        self.w.shutdown.clicked.connect(self.shutdown)
        self.w.reboot.clicked.connect(self.reboot)
        self.w.accesspoint.clicked.connect(self.accesspoint)
        self.w.touch_calibrate.clicked.connect(self.touch_calibrate)
        self.w.updatedrawing.clicked.connect(self.updatedrawing)
        self.w.zoom.valueChanged.connect(self.zoomDrawings)
        #self.w.tempImpostata.valueChanged.connect(self.setTempImp)
        self.w.cavename.textChanged.connect(self.cavenamechanged)
        self.requiredData = {}
        self.section = [0.0 for deg in range(360)]  #we expect to get one value for every degree
        self.walls = {}
        self.mycfgfile = QDir.homePath() + "/.charlottecfg"
        self.myCaveFile = {}
        self.myCoordinates = {}
        self.branches = []
        self.mycfg = {}
        self.newCave()
        #Take note that Y axis on a graphicsview is flipped
        self.w.pianta.scale(1,-1)
        self.w.spaccato.scale(1,-1)
        self.w.section.scale(1,-1)
        self.w.section.scale(20,20)
        self.csvheader = ["From", "To", "SideTilt", "FrontalInclination", "heading", "distance", "left", "right", "up", "down"]
        print("UI loaded")
        QApplication.processEvents()
        self.loadPersonalCFG()
        if os.path.isfile(self.mycfg['lastcave']) and self.mycfg['startfromlastcave']=='True':
            self.openFile()
        if isRPI:
            self.getDataThread = getData(self)
            self.getDataThread.start()
        self.startLidarScan()
        self.firstdistance = 0.0

    #TODO: eventfilter for keypad https://stackoverflow.com/questions/27113140/qt-keypress-event-on-qlineedit

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

    def calculateSectionCoord(self, section, myCenter = [0.0, 0.0, 0.0], heading = 0.0, incl = 0.0, sidetilt = 0.0):
        if len(section) <360:
            return None
        coords = []
        for angle in range(360):
            tmpangle = 360-(angle-sidetilt)
            d = section[angle]
            x = - d*math.cos(math.radians(tmpangle))
            y = 0
            z = d*math.sin(math.radians(tmpangle))

            #This code seems to be slower than numpy
            #rotate by heading
            #hCorr = 90
            #xold = -x
            #yold = y
            #x = (xold*(math.cos(math.radians(heading+hCorr)))) - (yold*(math.sin(math.radians(heading+hCorr))))
            #y = (xold*(math.sin(math.radians(heading+hCorr)))) + (yold*(math.cos(math.radians(heading+hCorr))))
            #rotate by incl
            #iCorr = 0.0
            #yold = y
            #zold = z
            #y = (yold*(math.cos(math.radians(incl+iCorr)))) - (zold*(math.sin(math.radians(incl+iCorr))))
            #z = (yold*(math.sin(math.radians(incl+iCorr)))) + (zold*(math.cos(math.radians(incl+iCorr))))

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

            #Rotate by inclination
            theta = np.radians(incl)
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

            #Translate by center
            x = x + myCenter[0]
            y = y + myCenter[1]
            z = z + myCenter[2]

            #add values to coords list
            coords.append([x,y,z])
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

    def saveFile(self, firstdistance = 0.0):
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
        Xfilename = self.mycfg["outputfolder"] + "/" + cleanedname + "/" + cleanedname + ".csx"
        print("Saving to " + Cfilename)

        temp = self.w.temperature.value()
        press = self.w.pressure.value()

        dist = float(self.w.distance.value())
        if firstdistance > 0.0:
            dist = firstdistance - dist
        requiredData = {
        'sideTilt':self.w.sideTilt.value(),
        'frontalInclination':self.w.frontalInclination.value(),
        'heading':self.w.heading.value(),
        'distance':dist
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

        self.myCaveFile = Cfile

        #now we build the CSV based on the measurements
        tfiletxt = self.json2CSV(Cfile)
        text_file = open(Tfilename, "w", encoding='utf-8')
        text_file.write(tfiletxt)
        text_file.close()
        self.populateTable(tfiletxt)

        #now we build the cSurvey CSX file
        xfiletxt = self.json2CSX(Cfile)
        text_file = open(Xfilename, "w", encoding='utf-8')
        text_file.write(xfiletxt)
        text_file.close()

        #we draw the result
        self.updatedrawing()

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

    def json2CSX(self, Cfile):
        cavename = self.w.cavename.text()
        now = datetime.now()
        mydate = now.strftime("%d-%m-%Y")
        #header
        csxtxt = "<csurvey version=\"1.11\" id=\"\"> \n<!-- "+mydate+" created by Charlotte --> \n  <properties id=\"\" name=\"\" origin=\"0\" creatid=\"TopoDroid\" creatversion=\"5.0.3h\" creatdate=\""+now.strftime("%Y-%m-%d")+"\" calculatemode=\"1\" calculatetype=\"2\" calculateversion=\"-1\" ringcorrectionmode=\"2\" nordcorrectionmode=\"0\" inversionmode=\"1\" designwarpingmode=\"1\" bindcrosssection=\"1\"> \n    <sessions> \n      <session date=\""+now.strftime("%Y.%m.%d")+"\" description=\""+cavename+"\" nordtype=\"0\" manualdeclination=\"0\" > \n      </session> \n    </sessions>"
        #caveinfo
        csxtxt = csxtxt + "<caveinfos> \n      <caveinfo name=\""+cavename.upper()+"\" > \n        <branches> \n          <branch name=\"\"> \n          </branch> \n        </branches> \n      </caveinfo> \n    </caveinfos> \n    <gps enabled=\"0\" refpointonorigin=\"0\" geo=\"WGS84\" format=\"\" sendtotherion=\"0\" /> \n  </properties>"
        csxtxt = csxtxt + "\n<segments>"

        i = 0
        s = 5
        for row in Cfile["measurements"]:
            mydate = str(row["timestamp"].split(" ")[0]).replace("/", "")

            #csvtxt = csvtxt + str(row["topographic"]['sideTilt']) + ","

            csxtxt = csxtxt + "<segment id=\"\" cave=\""+cavename.upper()+"\" branch=\"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["from"]) +"("+str(i)+")\" cut=\"1\" splay=\"1\" exclude=\"1\" distance=\"1.00\" bearing=\"293.0\" inclination=\"0.0\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\"0\" r=\"0\" u=\"0\" d=\"0\" distox=\"\" >\n    </segment>"
            i = i + 1
            csxtxt = csxtxt + "<segment id=\"\" cave=\""+cavename.upper()+"\" branch=\"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["from"]) +"("+str(i)+")\" cut=\"1\" splay=\"1\" exclude=\"1\" distance=\"2.00\" bearing=\"113.0\" inclination=\"0.0\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\"0\" r=\"0\" u=\"0\" d=\"0\" distox=\"\" >\n    </segment>"
            i = i + 1
            csxtxt = csxtxt + "<segment id=\"\" cave=\""+cavename.upper()+"\" branch=\"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["from"]) +"("+str(i)+")\" cut=\"1\" splay=\"1\" exclude=\"1\" direction=\"2\" distance=\"3.00\" bearing=\"0.0\" inclination=\"90.0\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\"0\" r=\"0\" u=\"0\" d=\"0\" distox=\"\" >\n    </segment>"
            i = i + 1
            csxtxt = csxtxt + "<segment id=\"\" cave=\""+cavename.upper()+"\" branch=\"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["from"]) +"("+str(i)+")\" cut=\"1\" splay=\"1\" exclude=\"1\" direction=\"2\" distance=\"1.00\" bearing=\"0.0\" inclination=\"-90.0\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\"0\" r=\"0\" u=\"0\" d=\"0\" distox=\"\" >\n    </segment>"
            i = i + 1
            csxtxt = csxtxt + "<segment id=\""+str(s)+"\" cave=\""+cavename.upper()+"\" branch=\"\" session=\""+mydate+"_"+cavename+"\" from=\""+ str(row["from"]) +"\" to=\""+ str(row["to"]) +"\" distance=\""+ str(row["topographic"]['distance']) +"\" bearing=\""+ str(row["topographic"]['heading']) +"\" inclination=\""+ str(row["topographic"]['frontalInclination']) +"\" g=\"0.0\" m=\"0.0\" dip=\"0.0\" l=\""+ str(row["walls"]['left']) +"\" r=\""+ str(row["walls"]['right']) +"\" u=\""+ str(row["walls"]['up']) +"\" d=\""+ str(row["walls"]['down']) +"\" distox=\"\" >\n    </segment>"
            s = s + 5

        #end segments
        csxtxt= csxtxt + "</segments>"
        #footer
        csxtxt= csxtxt + "<trigpoints> \n  </trigpoints> \n  <plan> \n    <layers> \n      <layer name=\"Base\" type=\"0\"> \n         <items /> \n      </layer> \n      <layer name=\"Soil\" type=\"1\"> \n        <items /> \n      </layer> \n      <layer name=\"Water and floor morphologies\" type=\"2\"> \n        <items /> \n      </layer> \n      <layer name=\"Rocks and concretions\" type=\"3\"> \n        <items /> \n      </layer> \n      <layer name=\"Ceiling morphologies\" type=\"4\"> \n        <items /> \n      </layer> \n      <layer name=\"Borders\" type=\"5\"> \n        <items> \n        </items> \n      </layer> \n      <layer name=\"Signs\" type=\"6\"> \n        <items /> \n      </layer> \n    </layers> \n    <plot /> \n  </plan> \n  <profile> \n    <layers> \n      <layer name=\"Base\" type=\"0\"> \n         <items /> \n      </layer> \n      <layer name=\"Soil\" type=\"1\"> \n        <items /> \n      </layer> \n      <layer name=\"Water and floor morphologies\" type=\"2\"> \n        <items /> \n      </layer> \n      <layer name=\"Rocks and concretions\" type=\"3\"> \n        <items /> \n      </layer> \n      <layer name=\"Ceiling morphologies\" type=\"4\"> \n        <items /> \n      </layer> \n      <layer name=\"Borders\" type=\"5\"> \n        <items> \n        </items> \n      </layer> \n      <layer name=\"Signs\" type=\"6\"> \n        <items /> \n      </layer> \n    </layers> \n    <plot /> \n  </profile> \n</csurvey>"
        return csxtxt

    def Json2Svg(self, Cfile):
        print("Drawing SVG")
        #scenes
        spaccato = QGraphicsScene()
        pianta = QGraphicsScene()
        for branch in self.branches:
            Ppoligon = QPainterPath()
            Spoligon = QPainterPath()
            Ppath = QPainterPath()
            Spath = QPainterPath()
            left = []
            right = []
            up = []
            down = []
            Ppath.moveTo(QPointF(0, 0))
            Spath.moveTo(QPointF(0, 0))
            for point in branch:
                #print(point)
                myX = self.myCoordinates[point]["pos"][0]
                myY = self.myCoordinates[point]["pos"][1]
                myZ = self.myCoordinates[point]["pos"][2]
                Ppath.lineTo(QPointF(myX, myY))
                Spath.lineTo(QPointF(myY, myZ))
                #print(Ppath.currentPosition())
                lX = self.myCoordinates[point]["left"][0]
                lY = self.myCoordinates[point]["left"][1]
                lZ = self.myCoordinates[point]["left"][2]
                rX = self.myCoordinates[point]["right"][0]
                rY = self.myCoordinates[point]["right"][1]
                rZ = self.myCoordinates[point]["right"][2]
                uX = self.myCoordinates[point]["up"][0]
                uY = self.myCoordinates[point]["up"][1]
                uZ = self.myCoordinates[point]["up"][2]
                dX = self.myCoordinates[point]["down"][0]
                dY = self.myCoordinates[point]["down"][1]
                dZ = self.myCoordinates[point]["down"][2]
                left.append([lX,lY])
                right.append([rX,rY])
                up.append([uY,uZ])
                down.append([dY,dZ])
            Ppoligon.moveTo(QPointF(left[0][0], left[0][1]))
            Spoligon.moveTo(QPointF(up[0][0], up[0][1]))
            for p in left:
                c1 = Ppoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Ppoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
            right.reverse()
            right.append(left[0]) #close the line
            right = right[1:]
            for p in right:
                c1 = Ppoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Ppoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
            for p in up:
                c1 = Spoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Spoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
            down.reverse()
            down.append(up[0]) #close the line
            down = down[1:]
            for p in down:
                c1 = Spoligon.currentPosition()
                c2 = QPointF(p[0], p[1])
                Spoligon.cubicTo(c1, c2, QPointF(p[0], p[1]))
            PennaBordo = QPen(Qt.black, 1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin) #Qt.Dashline https://doc.qt.io/qtforpython/PySide2/QtGui/QPen.html
            pianta.addPath(Ppoligon, PennaBordo)
            spaccato.addPath(Spoligon, PennaBordo)
            PennaPoligonale = QPen(Qt.red)
            pianta.addPath(Ppath, PennaPoligonale)
            spaccato.addPath(Spath, PennaPoligonale)
        #show in graphicsview
        self.w.pianta.setScene(pianta)
        self.w.spaccato.setScene(spaccato)
        self.w.pianta.show()
        self.w.spaccato.show()
        #files
        cleanedname = self.cleanName(self.w.cavename.text())
        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        Pfilename = cavefolder + "/" + cleanedname + "-pianta.svg"
        Sfilename = cavefolder + "/" + cleanedname + "-spaccato.svg"
        self.saveSvg(pianta, Pfilename, "Pianta")
        self.saveSvg(spaccato, Sfilename, "Spaccato")

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
        myCenter = [0.0, 0.0, 0.0]
        secCoords = self.calculateSectionCoord(section, myCenter, 0.0, 0.0, sideTilt)
        if len(secCoords) <360:
            return
        sezione = QGraphicsScene()
        PennaBordo = QPen(Qt.black, 0.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin) #Qt.Dashline https://doc.qt.io/qtforpython/PySide2/QtGui/QPen.html
        Spoligon = QPainterPath()
        Spoligon.moveTo(QPointF(secCoords[0][0], secCoords[0][2]))
        for angle in range(1,len(secCoords)):
            c1 = Spoligon.currentPosition()
            c2 = QPointF(secCoords[angle][0], secCoords[angle][2])
            Spoligon.cubicTo(c1, c2, QPointF(secCoords[angle][0], secCoords[angle][2]))
        c1 = Spoligon.currentPosition()
        c2 = QPointF(secCoords[0][0], secCoords[0][2])
        Spoligon.cubicTo(c1, c2, QPointF(secCoords[0][0], secCoords[0][2]))
        sezione.addPath(Spoligon, PennaBordo)
        if mysection == None:
            self.w.section.setScene(sezione)
            self.w.section.show()
            #QApplication.processEvents()
        else:
            return sezione

    def saveSvg(self, scene, fileName, title = ""):
        generator = QSvgGenerator()
        generator.setFileName( fileName )
        generator.setSize( QSize( scene.width(), scene.height() ) )
        generator.setTitle(title);
        generator.setDescription("Created with Charlotte Cave Surveing Software");
        painter = QPainter( generator )
        transform = QTransform().scale(1, -1) #we need a qtransform because in a qgraphicscene y-axis is flipped
        transform.translate(0, -scene.height())
        painter.setTransform(transform)
        scene.render( painter )
        painter.end()

    def zoomDrawings(self, val):
        zoom = self.w.zoom.value()/10
        self.Json2Svg(self.myCaveFile)
        self.w.pianta.scale(zoom,zoom)
        self.w.spaccato.scale(zoom,zoom)

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
            msp.add_line((fromX, fromY, fromZ), (toX, toY, toZ))  # add a LINE entity
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
                    msp.add_line((sFx, sFy, sFz), (sTx, sTy, sTz))
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
                            mesh_data.add_face([(aX,aY,aZ), (cX,cY,cZ), (dX,dY,dZ), (bX,bY,bZ)])
                    else:
                        with mesh.edit_data() as mesh_data:
                            mesh_data.add_face([(aX,aY,aZ), (toX,toY,toZ), (bX,bY,bZ)])
                mesh_data.optimize()  # optional, minimizes vertex count
        cleanedname = self.cleanName(self.w.cavename.text())
        cavefolder = self.mycfg["outputfolder"] + "/" + cleanedname
        Dfilename = cavefolder + "/" + cleanedname + ".dxf"
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
        self.Json2Dxf(self.myCaveFile)
        #xfiletxt = self.json2CSX(self.myCaveFile)
        #print(xfiletxt)

    def getFromPointData(self, Cfile, pointname):
        point = {}
        for row in Cfile["measurements"]:
            if row["from"] == pointname:
                point = row
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
            found = False
            for row in Cfile["measurements"]:
                Tpoint = row["to"]
                Fpoint = row["from"]
                #we might already have considered this frompoint but coupled with another Tpoint
                if Fpoint==startpoint and not self.checkPointBranched(rami, Tpoint):
                    rami[r].append(Fpoint)
                    startpoint = Tpoint
                    found = True
                if bool(Tpoint==startpoint and not self.checkPointBranched(rami, Tpoint)) and not found:
                    rami[r].append(Tpoint)
                    startpoint = Tpoint
                    found = True
            if not found:
                #if this point is a "to" but not a "from", means we reached the end of this branch
                r = r +1
            #go on until there is a point still not considered
            startpoint = self.checkPointNotBranched(rami)
        self.branches = rami

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
            myX = fromX + (dist*(math.cos(math.radians(heading))))
            myY = fromY + (dist*(math.sin(math.radians(heading))))
            myZ = fromZ + (dist*(math.sin(math.radians(incl))))
            coord[pointname] = self.getPointWalls(pointname, frompointname, myX, myY, myZ)
        self.myCoordinates = coord

    def getPointWalls(self, pointname, frompointname, myX, myY, myZ, Cfile = None):
        pcoords = {}
        if Cfile == None:
            Cfile = self.myCaveFile
        rawdata = self.getFromPointData(Cfile, frompointname)
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
        rawdata = self.getFromPointData(Cfile, pointname)
        if len(rawdata) >0:
            l = rawdata["walls"]['left']
            r = rawdata["walls"]['right']
            u = rawdata["walls"]['up']
            d = rawdata["walls"]['down']
            lX = myX + (l*(math.cos(math.radians(heading+90))))
            lY = myY + (l*(math.sin(math.radians(heading+90))))
            lZ = myZ
            rX = myX + (r*(math.cos(math.radians(heading-90))))
            rY = myY + (r*(math.sin(math.radians(heading-90))))
            rZ = myZ
            uX = myX
            uY = myY + (u*(math.sin(math.radians(heading+90))))
            uZ = myZ + (u*(math.sin(math.radians(incl+90))))
            dX = myX
            dY = myY + (d*(math.sin(math.radians(heading-90))))
            dZ = myZ + (d*(math.sin(math.radians(incl-90))))
            pcoords = {"pos":[myX,myY,myZ],"left":[lX,lY,lZ],"right":[rX,rY,rZ],"up":[uX,uY,uZ],"down":[dX,dY,dZ]}
        else:
            pcoords = {"pos":[myX,myY,myZ],"left":[myX,myY,myZ],"right":[myX,myY,myZ],"up":[myX,myY,myZ],"down":[myX,myY,myZ]}
        return pcoords

    #def getPointSection(self, pointname, frompointname, myX, myY, myZ, Cfile = None):
    #    print("Calculating section points")
    #    scoords = []
    #    return scoords

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
        self.myCaveFile = Cfile

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
            self.saveFile()
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
        if self.w.puntofisso.isChecked():
            th = threading.Thread(target=self.puntofissoSave) #args=self.firstdistance
            th.start()
        #else:
        #    th.stop()

    def puntofissoSave(self):
        #rm /home/luca/charlottedata/prova1/prova1.json && touch /home/luca/charlottedata/prova1/prova1.json
        active = True
        firstdistance = float(self.w.distance.value())
        while active:
            if self.w.puntofisso.isChecked():
                self.w.puntofisso.setStyleSheet("background-color: rgb(0, 255, 0);")
                self.saveFile(firstdistance)
                if self.mycfg["lastcave"] == "":
                    print("Error: lastcave is null")
                    return
                #increment from and to
                self.incrementFromTo()
                self.w.puntofisso.setStyleSheet("background-color: rgb(127, 127, 127);")
                QApplication.processEvents()
            else:
                active = False
                break
            startTS = datetime.now().timestamp()
            toWait = 2
            toSleep = 0.1
            print("Wait 2 seconds")
            while (datetime.now().timestamp()-startTS) < toWait:
                time.sleep(toSleep)
        print("Stopping autosave timer")
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
