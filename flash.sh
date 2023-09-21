#./erase_wifi_pw.sh
#./upload_certs.sh
idf.py -p /dev/cu.usbmodem14101 flash 
idf.py -p /dev/cu.usbmodem14101 monitor
