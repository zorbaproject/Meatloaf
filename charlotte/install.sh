#!/bin/bash

sudo apt-get update
sudo apt install aptitude

sudo aptitude install python3-rpi.gpio
sudo pip3 install w1thermsensor

sudo apt-get install i2c-tools python-smbus
sudo ./setup_i2c.sh
echo "Please enable SPI and I2C from Interfacing Options"
read
sudo raspi-config
sudo pip3 install adafruit-circuitpython-lsm303-accel
sudo pip3 install adafruit-circuitpython-lsm303dlh-mag

sudo apt install libpyside2-dev pyside2-tools
sudo apt install python3-pyside2*
#Minimalistic option: sudo apt install python3-pyside2.qtgui python3-pyside2.qtcore python3-pyside2.qtconcurrent

wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/adafruit-pitft.sh
chmod +x adafruit-pitft.sh
echo "Please enable your LCD TFT module (Suggested options: 5, 1, N, Y, N)"
read
sudo ./adafruit-pitft.sh

#Run autologin to make app run at boot
chmod +x autologin.sh
sudo ./autologin.sh

sudo apt clean

echo "Now just reboot the system and wait for the interface to load."
