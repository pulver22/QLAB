#!/bin/bash
# Shortcut for publishing twist commands
echo $#
if [ "$#" -lt 1 ]; then 
    echo "No args, so setting all velocities to zero"
    dvel="0.0"
    dyaw="0.0"

elif [ "$#" -lt 2 ];then
    echo "Only one arg, so setting only fwd vel"
    dvel=$1
    dyaw="0.0"
else
    echo "Setting both fwd and yaw rates"
    dvel=$1
    dyaw=$2
fi

#echo "Vel = ${dvel}, Yaw-rate = ${dyaw}"
TOPIC="p5/my_p3at/cmd_vel"
#TOPIC="nav_cmd_vel" # for testing
cmd="rostopic pub -r 10 /${TOPIC} geometry_msgs/Twist  '{linear:  {x: ${dvel}, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: ${dyaw}}}'"

echo ${cmd}

eval "${cmd}"

