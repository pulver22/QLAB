#!/bin/bash
#rosrun rqt_plot rqt_plot /pid_debug/D:Derivative:Error:P &
rosrun rqt_plot rqt_plot /pid_debug/P:I:D:PID &
rosrun rqt_plot rqt_plot /pid_debug/Error &
