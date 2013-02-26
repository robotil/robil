#!/bin/sh
#This script should run the modules needed for testing C31

CURR_WD=$PWD
PKILL=$CURR_WD'/.killC31test.txt'
rm $PKILL
echo "Running roscore"
roscore &
ROS_PID=$!
echo $ROS_PID
sleep 10
rosrun C22_GroundRecognitionAndMapping C22 &
C22_PID=$!
echo $C22_PID
sleep 3
echo "Playing bag file"
rosbag play record_points4_2013-01-14-14-22-01.bag 
echo "Starting executer"
rosrun C34_Executer executer &
C34_PID=$!
echo $C34_PID
echo "Done "
sleep 3
echo "Starting C31"
rosrun C31_PathPlanner gpp&
C31_PID=$!
echo $C31_PID
echo "Done "
echo $ROS_PID $C22_PID $C31_PID $C34_PID >> $PKILL

