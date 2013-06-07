#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.perception.txt'
rm $PKILL
echo "Starting C21"
roslaunch C21_VisionAndLidar C21.launch &
C21_PID=$!
echo $C21_PID
sleep 3
echo "Starting C25"
roslaunch C25_GlobalPosition              C25.launch &
C25_PID=$!
echo $C25_PID
echo "Starting C22"
roslaunch C22_GroundRecognitionAndMapping C22_Cheats.launch &
C22_PID=$!
echo $C22_PID
sleep 3
echo $C21_PID $C22_PID $C25_PID >> $PKILL
