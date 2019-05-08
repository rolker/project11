# Operating the CW4 using the AutonomousMissionPlanner via ROS.

A brief tutorial on bringing up the ROS environment on CCOM/JHC's CW4.

## Prerequisites

  * CW4 is powered up as well as payload computer mystique.
  * Operator workstation (penguin) is powered up and connected to VLAN100 which is the Cobham/MBR network.
    * Operator station has host file with relevant IP addresses

## Connect to Mystique

In a terminal on the operator station, open an ssh connection to mystique over the Cobham with X forwarding.

    ssh -X mystiquec
  
This single session can be used to open new xterms as necessary.

    xterm &

### Launch ROS on Mystique

#### Updating to tmux

Detaching...

    ctrl+b d    detach
    
Split panes:

    ctrl+b "    horizontal
    ctrl+b %    vertical

Move to panes:

    ctrl+b [arrow key]
    
List sessions:
    tmux ls

Attach to named session:
    tmux a -t [name of session]

Kill current pane:
    ctrl+b x

#### new tmux and rosmon instructions

To allow the output to be viewed, yet remain robust to disconnects, tmux is used to launch ROS.

    tmux new -s ros
    roscore
    ctrl+b "
    mon launch project11 mystique.launch
    
To detach and reattach

    ctrl+b d
    tmux a -t ros

#### old screen and roslaunch instructions

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

    roslaunch project11 operator_mystique_cobham.launch


### AutonomousMissionPlanner

The executable for the mission planner should be in the build directory of the source package, which may be in ~/src or ~/gitsrc.

    ~/src/AutonomousMissionPlanner/build/AutonomousMissionPlanner
    
Open up an appropriate background chart by clicking the BG button and navigating to the desired subfolder in BSB_ROOT and selecting a .KAP file.

Create a TrackLine (TL), Waypoint (WP), or Survey Pattern (SP) and send it to ROS by rightclicking on the item in the list on the left and selecting "Send To ROS".

To have the boat execute the mission, make sure control is givien to the backseat driver in ASView-Bridge and the vehicle state is Active.

### Dynamic Reconfigure

The reconfigure gui can be launched from the operator station to connect to mystique to allow parameters to be cahnged on the fly.

    project11/scripts/Project11_ROS_Reconfigure.bash
    
### Debugging

On mystique, you can watch the commands sent to the boat by looking at the heading_hold topic.

    rostopic echo /control/drive/heading_hold
    
    
