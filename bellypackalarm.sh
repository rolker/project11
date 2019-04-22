#!/bin/bash
# THIS IS NOT WORKING :(
while [ 1 ] ; do

    ping 192.168.100.33 || tput bel
    #if [ ! `ping bellypackhelmw ` ]; then
    #if [ ! ping 192.168.100.33 ]; then
    #    tput bel
    #fi
    sleep 1
done
