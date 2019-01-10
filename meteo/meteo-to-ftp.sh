#!/bin/sh
#crontab -e
#0,10,20,30,40,50 * * * * /home/pi/meteo-to-ftp.sh
HOST='ftp.kraskikrti.net'
USER='1226041@aruba.it'
PASSWD=$(cat passwd.txt)
FOLDER='www.kraskikrti.net/meteo/'

localtmp="/var/www/html/meteo/"
cd $localtmp

FILE="logs.txt"
ftp -n $HOST <<END_SCRIPT
quote USER $USER   
quote PASS $PASSWD
binary     
cd $FOLDER
put "$localtmp$FILE" "$FILE"
quit
END_SCRIPT
exit 0


