Network: 192.168.88.x

penguin (operator station): 192.168.88.42
    user: field

zboat (backseat driver): 192.168.88.7
    ssh sreed@192.168.88.7 multibeam
    

Instructions from operator station

- Open a terminal for connecting to the backseat driver
- ssh to the zboat (see above)
- use the screen command to open a detachable session
- roslaunch project11 echo.launch

- Open a terminal for local ros stuff
- roslaunch project11 operator_echo.launch

- Open a terminal for AMP
- cd src/AutonomousMissionPlanner/build
- ./AutonomousMissionPlanner

- Open a terminal for jostick sender
- ./project11/catkin_ws/src/remote_control/scripts/remote_control_joy_sender.py address=192.168.88.7

Notes:

- red button on joypad to disable joypad
- green button on joypad to enable joypad
- if joypad is enabled, it overrides MOOS
- enabled must be checked in AMP for vehicle to listen to joypad or MOOS
- when taking over with joypad, recomend setting mode to Standby instead of Survey in AMP. If left in Survey mode, vehicle might try to resume mission when joypad disabled


