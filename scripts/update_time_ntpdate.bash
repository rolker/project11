#!/bin/bash

service ntp stop
ntpdate time.unh.edu
service ntp start

