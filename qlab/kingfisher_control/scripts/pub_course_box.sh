#!/bin/bash
TOPIC="cmd_course"

# Yaw setpoints as an array
Yaws=(0.0 \
    1.57
    3.14
    -3.14
    -1.57
    0.0)
vel=1.0

DT=3 # Seconds to sleep between commands

# For each element in the array
for yaw in "${Yaws[@]}"
do
    cmd="rostopic pub -1 /${TOPIC} kingfisher_msgs/Course  '{speed: $vel, yaw: $yaw}'"
    
    echo ${cmd}
    eval "${cmd}"
    sleep $DT
done

vel=0.0
yaw=0.0
cmd="rostopic pub -1 /${TOPIC} kingfisher_msgs/Course  '{speed: $vel, yaw: $yaw}'"
echo ${cmd}
eval "${cmd}"
sleep $DT
