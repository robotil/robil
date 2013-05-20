#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.perception.txt'
rm $PKILL
echo "Starting C21"
rosrun C21_VisionAndLidar C21 &
C21_PID=$!
echo $C21_PID
sleep 3
echo "Starting C22"
rosrun C22_GroundRecognitionAndMapping C22 &
C22_PID=$!
echo $C22_PID
sleep 3
echo "Starting C24"
rosrun C24_ObstacleDetection           C24 &
C24_PID=$!
echo $C24_PID
sleep 3
echo "Starting C25"
roslaunch C25_GlobalPosition              C25.launch &
C25_PID=$!
echo $C25_PID
echo $C21_PID $C22_PID $C24_PID $C25_PID >> $PKILL
