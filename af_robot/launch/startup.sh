#!/bin/bash

if [ "`pgrep roslaunch`" != "" ]
then
  echo Startup roslaunch already running > ~/catkin_ws/src/af_robot/startup.log
  exit 2
fi

count=0
while [ "`ip -f inet -o addr show | grep 10.0.1.100 | cut -d\  -f7 | cut -d/ -f1`" = "" ]
do
  sleep 1
  count=$[$count + 1]
  if [ $count -ge 600 ]
  then
    echo IP lookup failed！> ~/catkin_ws/src/af_robot/startup.log
    exit 1
  fi
done

ip=`ip -f inet -o addr show | grep 10.0.1.100 | cut -d\  -f7 | cut -d/ -f1`
echo IP address
export ROS_HOSTNAME=$ip
echo $ROS_HOSTNAME
echo ROS_MASTER_URI
export ROS_MASTER_URI=http://$ip:11311
echo $ROS_MASTER_URI
echo ======================================
echo `date`

gnome-terminal -x bash -c "roslaunch af_robot function_demo.launch"
#roslaunch 
#roscore
#gnome-terminal -x bash -c "roscore"

