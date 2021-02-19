#!/usr/bin/env python3

import serial
import datetime
import socket


# to start from crontab on reboot, add the following using crontab -e
#
# @reboot python3 /home/ubuntu/project11/scripts/aishub_sender.py >>/home/ubuntu/aishub_sender_log.txt 2>&1



# input, serial port stuff

serial_port = '/dev/ttyUSB0'
serial_speed = 38400


input = serial.Serial(serial_port, serial_speed, timeout=1)

# log to file

log_prefix = '/home/ubuntu/ais-logs/ais-log-'

today = None

# stream to udp destinations

udp_destinations = (('144.76.105.244',2100), #aishub
                    ('5.9.207.224',11566),   #marinetraffic
                    ('109.200.19.151',4001), #shipfinder.co
                    ('148.251.96.197',34987),#fleetmon
                    ('10.242.236.198',2125), #snowpetrelz
                    ('10.242.46.146',2125) ) #penguinz
udp_sockets = []
for d in udp_destinations:
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sockets.append((s,d))

while True:
  line_bin = input.readline()
  if len(line_bin):
    now = datetime.datetime.utcnow()
    line = line_bin.decode('ascii')
    #print (now.isoformat(),line)

    #rotate log files daily (utc time)
    if today != now.date().isoformat():
      today = now.date().isoformat()
      log_file = open(log_prefix+today+'.log','a')

    log_file.write(now.isoformat()+','+line)
    log_file.flush()

    for sock,addr in udp_sockets:
      try:
        sock.sendto(line_bin, addr)
      except Exception as e:
        pass
        #print('error sending to',addr,e)
