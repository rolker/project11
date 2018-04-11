# Operating the CW4 using the AutonomousMissionPlanner via ROS.

A brief tutorial on bringing up the ROS environment on CCOM/JHC's CW4.

## Prerequisites

  * CW4 is powered up as well as payload computer mystique.
  * Operator workstation (Ubuntu) is powered up and connected to VLAN100 which is the Cobham network.
    * (This document was first develop using an Ubuntu 16.04 virtual machine on the Windows machine blacktip.)
    * Operator station has host file with relevant IP addresses

## Connect to Mystique

In a terminal on the operator station, open an ssh connection to mystique over the Cobham with X forwarding.

  ssh -X mystiquec
  
This single session can be used to open new xterms as necessary.

  xterm &
  
To allow the output to be viewed, yet remain robust to disconnects, screen is used to launch ros.

