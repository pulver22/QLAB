#!/bin/bash
rostopic pub -1 /set_setpoint std_msgs/Float64 -- $1
