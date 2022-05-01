#!/bin/bash

echo "This install script is intended for a RaspberryPi 3"

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
sudo pip3 install Adafruit-BMP

sudo apt install libpyside2-dev pyside2-tools
sudo apt install python3-pyside2*
#Minimalistic option: sudo apt install python3-pyside2.qtgui python3-pyside2.qtcore python3-pyside2.qtconcurrent

#export in Autocad DXF format
sudo pip3 install ezdxf

#support for kml
sudo pip3 install -U pip
sudo pip3 install simplekml
sudo pip3 install pyproj

#access serial ports (e.g.: /dev/ttyUSB0) as non root
sudo pip3 install pyserial
sudo usermod -a -G dialout $(whoami)

wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/adafruit-pitft.sh
chmod +x adafruit-pitft.sh
echo "Please enable your LCD TFT module (Suggested options: 5, 1, N, Y, N)"
read
sudo ./adafruit-pitft.sh

#Run autologin to make app run at boot
chmod +x autologin.sh
sudo ./autologin.sh

#needed for the access point mode
chmod +x accesspoint.sh
sudo apt-get install iw hostapd wireless-tools dnsmasq

#boot configuration
cp ./config.txt /boot/config.txt

cat << EOF | tee /tmp/dhcpcd.conf
interface wlan0
static ip_address=192.168.1.1/24
denyinterfaces eth0
denyinterfaces wlan0
EOF
sudo mv /tmp/dhcpcd.conf /etc/dhcpcd.conf

sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
cat << EOF | tee /tmp/dnsmasq.conf
interface=wlan0
  dhcp-range=192.168.1.1,192.168.1.30,255.255.255.0,24h
  address=/charlotte.lan/192.168.1.1 # Alias for this router
EOF
sudo mv /tmp/dnsmasq.conf /etc/dnsmasq.conf

cat << EOF | tee /tmp/hostapd.conf
interface=wlan0
country_code=IT
#bridge=br0
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
#wpa=2
#wpa_key_mgmt=WPA-PSK
#wpa_pairwise=TKIP
#rsn_pairwise=CCMP
#wpa_passphrase=Talpe
ssid=CharlotteCaveSurveing
EOF
sudo mv /tmp/hostapd.conf /etc/hostapd/hostapd.conf
echo ""
echo "**Press Ctrl+C when finished testing hostapd**"
sudo hostapd /etc/hostapd/hostapd.conf

#webserver
sudo apt install apache2 libapache2-mod-php php php-mysql w3m
#Display the charlotte folder instead of usual directory
#sudo cp /etc/apache2/sites-available/000-default.conf /etc/apache2/sites-available/000-default.conf.old
#sudo sed -i "s/DocumentRoot.*/DocumentRoot \/home\/$(whoami)\/charlottedata/g" /etc/apache2/sites-available/000-default.conf
sudo systemctl enable apache2
sudo systemctl start apache2
sudo mkdir "/home/$(whoami)/charlottedata"
sudo chown -R pi:www-data "/home/$(whoami)/charlottedata"
sudo chmod -R 777 "/home/$(whoami)/charlottedata"
sudo chmod -R 777 "/home/$(whoami)"
sudo adduser pi www-data
sudo adduser www-data pi
echo "Options +Indexes" >> "/home/$(whoami)/charlottedata/.htaccess"
sudo mv /var/www/html /var/www/htmlOLD
sudo ln -s "/home/$(whoami)/charlottedata" /var/www/html

#rtc
sudo bash -c 'echo "rtc-ds1307" >> /etc/modules'
sudo cp rc.local /etc/rc.local

#hostname
sudo hostname charlotte

sudo apt clean

echo "Now just reboot the system and wait for the interface to load."
