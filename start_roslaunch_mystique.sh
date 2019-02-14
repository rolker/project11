# !/bin/bash

# inspired by: https://answers.ros.org/question/140426/issues-launching-ros-on-startup/

LOG_FILE=/home/field/project11/log/roslaunch.txt

echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running start_roslaunch_mystique.bash" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

set -e

{
source /opt/ros/kinetic/setup.bash
source /home/field/project11/catkin_ws/devel/setup.bash

export ROS_WORKSPACE=/home/field/project11/catkin_ws


} &>> ${LOG_FILE}

set -v

{

roslaunch project11 mystique.launch

} &>> ${LOG_FILE}
