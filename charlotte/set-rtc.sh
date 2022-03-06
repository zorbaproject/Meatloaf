#!/bin/bash
sudo nptd -g -q
date
sudo hwclock -r
sudo hwclock -w
sudo hwclock -s

#http://www.intellamech.com/RaspberryPi-projects/rpi_RTCds3231
