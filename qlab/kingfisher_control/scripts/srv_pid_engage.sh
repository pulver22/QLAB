#!/bin/bash
echo $#
EBOOL="TRUE"
if [ "$#" -lt 1 ]; then 
    echo "No args, setting engage to true"
    EBOOL="True"
else
    EBOOL=$1
fi

rosservice call /set_engaged ${EBOOL}
