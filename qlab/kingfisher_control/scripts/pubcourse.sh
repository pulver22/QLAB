#!/bin/bash
# Shortcut for publishing twist commands
echo $#
if [ "$#" -lt 1 ]; then 
    echo "No args, so setting all to zero"
    dvel="0.0"
    dyaw="0.0"

elif [ "$#" -lt 2 ];then
    echo "Only one arg, so setting only fwd vel"
    dvel=$1
    dyaw="0.0"
else
    echo "Setting both fwd and yaw "
    dvel=$1
    dyaw=$2
fi

#echo "Vel = ${dvel}, Yaw-rate = ${dyaw}"
TOPIC="cmd_course"
#TOPIC="nav_cmd_vel" # for testing
cmd="rostopic pub -1 /${TOPIC} kingfisher_msgs/Course  '{speed: $dvel, yaw: $dyaw}'"

echo ${cmd}

eval "${cmd}"

