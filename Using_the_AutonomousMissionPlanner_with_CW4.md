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

### Launch ROS on Mystique

To allow the output to be viewed, yet remain robust to disconnects, screen is used to launch ROS.

    screen
    
Hit enter to get past the splash screen. Once you reach the command prompt, launch ROS on mystique.
    
    roslaunch project11 mystique.launch

#### Detaching screen (optional)

If the connection is lost, the screen will automatically detach and the process will keep on running in the background.

You can manually detach from the screen by hiting the sequence Ctrl-a, d.

Reattatching to the screen is possible with the -r switch.

    screen -r

## Operator station

On the operator station, open a terminal and launch the operator side of the UDP bridge.

    roslaunch udp_bridge operator.launch host:=mystiquec
    
The launch file defaults to localhost so the host parameter is specified. (mystiquec: Mystique over Cobham.)

One way to verify that all systems are communicating is with appcast_view. In a new terminal start it with rosrun.

    rosrun appcast_view appcast_view.py
    
Resize the terminal window appropriately. This displays output from the MOOS process running on Mystique.

### AutonomousMissionPlanner

The executable for the mission planner should be in the build directory of the source package, which may be in ~/src or ~/gitsrc.

    ~/gitsrc/AutonomousMissionPlanner/build/AutonomousMissionPlanner
    
Open up an appropriate background chart by clicking the BG button and navigating to the desired subfolder in BSB_ROOT and selecting a .KAP file.

Connect to ROS by hitting the ROS button.

A circle should appear at the location of the boat and a cross at the origin of the map frame.

Create a TrackLine (TL), Waypoint (WP), or Survey Pattern (SP) and send it to ROS by rightclicking on the item in the list on the left and selecting "Send To ROS".

To have the boat execute the mission, make sure control is givien to the backseat driver in ASView-Bridge and the vehicle state is Active. In the mission planner, make sure ROS is selected in the list to display the controls. Make sure the Active checkbox is checked and the Survey button is clicked to have MOOS execute the sent mission. When manipulating those buttons and checkbox, the output from appcast_view should change slightly. 
    
    
    
