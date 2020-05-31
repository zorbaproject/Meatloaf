#!/usr/bin/python3
#https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding
#https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/python-circuitpython
#https://tutorials-raspberrypi.com/measuring-rotation-and-acceleration-raspberry-pi/

from time import sleep
import math

import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
 
i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

#https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/calibration
#https://github.com/praneshkmr/node-lsm303/wiki/Understanding-the-calibration-of-the-LSM303-magnetometer-%28compass%29
#we could use this to get offsets and use them to map actual value to theoretical values
#https://gist.github.com/ViennaMike/d8b8f9636694c7edf4f115b28c9378c0

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)


def get_heading(x,y,z):
    if math.fabs(z) < 3:
        print("Tilting too much, heading measurement could be unaccurate")
    radians = math.atan2(y,x)
    heading = math.degrees(radians)
    if heading <0:
        heading = heading + 360
    return heading

while True:
    mag_x, mag_y, mag_z = mag.magnetic
    accel_x,accel_y,accel_z = accel.acceleration
    print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
    print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
    print("Rotation X (Side Tilt): "+str(get_x_rotation(accel_x,accel_y,accel_z)))
    print("Rotation Y (Frontal Inclination): "+str(get_y_rotation(accel_x,accel_y,accel_z)))
    print("Heading: "+str(get_heading(mag_x,mag_y,mag_z)))
    #Note: heading is measured from the X axis as drawn on the chip, increasing clockwise
    #Y rotation is the frontal inclination
    #X rotation is useful for cave section measurements
    print("")
    sleep(0.5)

