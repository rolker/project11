Network: 192.168.88.x

penguin (operator station): 192.168.88.42
    user: field

zboat (backseat driver): 192.168.88.7
    ssh -X sreed@192.168.88.7 multibeam
    

Instructions from operator station

- Open a terminal for connecting to the backseat driver
- ssh to the zboat (see above)
- use the `screen` command to open a detachable session 
- roslaunch project11 echo.launch
- [optional] Detach the `screen` session with `Ctrl-a d`. Reattach it with `screen -r`

- Open a terminal for local ros stuff
- roslaunch project11 operator_echo.launch

- Open a terminal for AMP
- cd src/AutonomousMissionPlanner/build
- ./AutonomousMissionPlanner

- Open a terminal for jostick sender
- ~/project11/catkin_ws/src/remote_control/scripts/remote_control_joy_sender.py address=192.168.88.7

Notes:

- The advantage of a detachable session, is that if the telemetry link is lost, ROS will continue to run. Otherwise, the loss of the ssh session will kill the ROS core. 
- red button on joypad to disable joypad
- green button on joypad to enable joypad
- if joypad is enabled, it overrides MOOS
- enabled must be checked in AMP for vehicle to listen to joypad or MOOS
- when taking over with joypad, recomend setting mode to Standby instead of Survey in AMP. If left in Survey mode, vehicle might try to resume mission when joypad disabled
- NUC and Operator's station must be in time synch (<1s) for the project11 pilot to accept joystick commands. When both are connected to the Internet, synch them by running `sudo ntpdate <server>` where <server> is a network time server (e.g. time.unh.edu). When they are not connected to the internet, ensure one of them is running an ntp server (sudo service ntp start), and run the ntpdate command above replacing <server> with the other's ip address. 
- Joystick commands are sent directly to the boat via UDP and published as /cmd_vel.
- Echo-boat commands are sent to the PixHawk FCU on /mavros/setpoint_velocity/cmd_vel.
- Monitor ROS topics with `rostopic echo <topic name>`.

