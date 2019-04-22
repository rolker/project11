#!/usr/bin/env python

import socket as S
import os

# Shutdown storm
print("Shutting down STORM")
s = S.socket(S.AF_INET,S.SOCK_DGRAM)
# Shutdown
s.sendto("SHUTDOWN",('stormc',9999))
os.system('ping -c 10 stormc')

# Shutdown mystiquec
print("Shutting down MYSTIQUE")
os.system('ssh -t mystiquec sudo shutdown -h now')
os.system('ssh -t mystiquew sudo shutdown -h now')
os.system('ping -c -w 3 mystiquec')


