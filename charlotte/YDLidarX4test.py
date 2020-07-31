#!/usr/bin/python3
#Based on: https://github.com/lakshmanmallidi/PyLidar3/blob/master/Examples/LidarTest.py

import PyLidar3
import time 

lidarport = "/dev/ttyUSB0"
scantime = 2


def findYDLidarX4(ttys = ["/dev/ttyUSB0", "/dev/ttyUSB1"]):
    try:
        for tmptty in ttys:
            Obj = PyLidar3.YdLidarX4(tmptty) #PyLidar3.your_version_of_lidar(port,chunk_size)
            if(Obj.Connect()):
                print(Obj.GetDeviceInfo())
                Obj.Disconnect()
                return tmptty
        t = 0/0
    except:
        return None

def scanYDLidarX4():
    global scantime
    global lidarport
    myscan = [0.0 for deg in range(360)] #we have one value for every angle
    Obj = PyLidar3.YdLidarX4(lidarport)
    if(Obj.Connect()):
        gen = Obj.StartScanning()
        t = time.time() # start time 
        scanlist = []
        while (time.time() - t) < scantime:
            scanlist.append(next(gen))
            time.sleep(0.5)
        Obj.StopScanning()
        Obj.Disconnect()
        for i in range(360):
            sum = 0.0
            for tmpscan in scanlist:
                sum = sum +tmpscan[i]
            myscan[i] = (float(sum)/len(scanlist))/1000.0
        print(len(scanlist))
    else:
        print("Error connecting to device")
    return myscan


lidarport = findYDLidarX4(["/dev/ttyUSB0", "/dev/ttyUSB1"])
print("Found Lidar on " + str(lidarport))
if lidarport != None:
    myscan = scanYDLidarX4()
    print(myscan)
