#!/bin/bash

sudo rfkill unblock 0  #remember to block it at next boot
sudo systemctl start dnsmasq
sudo systemctl unmask hostapd
sudo systemctl stop hostapd
#sudo hostapd /etc/hostapd/hostapd.conf
sudo systemctl start hostapd

#abilitiamo un samba share sulla cartella dei rilievi?

#enable traffic forwarding
#echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
#sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
#we should now bridge https://thepi.io/how-to-use-your-raspberry-pi-as-a-wireless-access-point/
