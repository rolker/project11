#!/bin/bash

LOG_FILE=$HOME/project11/log/start_tmux_operator.txt

echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running start_tmux_operator.bash" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e

{
source /opt/ros/melodic/setup.bash
source $HOME/project11/catkin_ws/devel/setup.bash

export ROS_WORKSPACE=$HOME/project11/catkin_ws
#export ROS_IP=192.168.100.199

} &>> ${LOG_FILE}

set -v

{
tmux new -d -s project11
tmux send-keys "roscore" C-m
tmux splitw -p 90
tmux send-keys "rosrun rosmon rosmon project11 operator_mystique_mobile_lab.launch" C-m
tmux splitw -p 50
tmux send-keys "rosrun rosmon rosmon project11 operator_ui.launch" C-m
} &>> ${LOG_FILE}
