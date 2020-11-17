#!/bin/bash

LOG_FILE=/home/field/project11/log/start_tmux.txt

echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running start_tmux.bash" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e

{
source /opt/ros/noetic/setup.bash
source /home/field/project11/catkin_ws/devel/setup.bash

export ROS_WORKSPACE=/home/field/project11/catkin_ws
export ROS_IP=192.168.100.112

} &>> ${LOG_FILE}

set -v

{
tmux new -d -s project11 roscore
tmux splitw -p 90
tmux send-keys "rosrun rosmon rosmon project11 ben.launch" C-m

} &>> ${LOG_FILE}
