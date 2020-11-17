# !/bin/bash

# called from cron @reboot using field's user crontab
# inspired by: https://answers.ros.org/question/140426/issues-launching-ros-on-startup/

LOG_FILE=/home/field/project11/log/autostart.txt

echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "Running autostart_mystique.bash" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}

#wait for mystique to be pingable by self
while ! ping -c 1 -W 1 mystique; do
    echo "Waiting for ping to mystique..."
    sleep 1
done

echo "Wait 10 seconds before launching ROS..."
sleep 10

set -e

{
source /opt/ros/noetic/setup.bash
source /home/field/project11/catkin_ws/devel/setup.bash

export ROS_WORKSPACE=/home/field/project11/catkin_ws
export ROS_IP=192.168.100.112

} &>> ${LOG_FILE}

set -v

{
/home/field/project11/scripts/start_tmux.sh
#screen -d -m bash /home/field/project11/scripts/start_roscore.sh
#screen -d -m bash /home/field/project11/scripts/start_rosmon_mystique.sh
} &>> ${LOG_FILE}
