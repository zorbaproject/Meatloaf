#!/bin/bash
shdir=$(cd -P -- "$(dirname -- "$0")" && pwd -P)
cd $shdir
git pull origin master
if [ -d "/var/www/html/meteo/" ]; then
mkdir /var/www/html/meteo/
fi
cp ./write-values.php /var/www/html/meteo/write-values.php
cp ./index.html /var/www/html/meteo/index.html

test=0
crontab -l | grep -q 'meteo-to-ftp.sh'  && test=1 || test=0
if [ $server == 0 ] ; then
(crontab -l 2>/dev/null ; echo "0,10,20,30,40,50 * * * * $shdir/meteo-to-ftp.sh") | crontab -
fi

test=0
crontab -l | grep -q 'install-meteo.sh'  && test=1 || test=0
if [ $server == 0 ] ; then
(crontab -l 2>/dev/null ; echo "0 * * * * $shdir/install-meteo.sh") | crontab -
fi
