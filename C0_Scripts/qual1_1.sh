#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.qual1.txt'
rm $PKILL

echo "Starting Perception"
roslaunch C25_GlobalPosition C25.launch &
C25_PID=$!
echo $C25_PID
sleep 3
echo "Starting C23"
rosrun C23_ObjectRecognition C23 /multisense_sl/camera/left/image_color /multisense_sl/camera/right/image_color /multisense_sl/camera/points2 &
C23_PID=$!
echo $C23_PID
sleep 3
rosrun C31_PathPlanner gpp > gpp.log &
C31_PID=$!
echo $C31_PID
echo "Starting DL"
rosrun C42_DynamicLocomotion DynamicLocomotion.py > dl.log &
C42_PID=$!
echo $C23_PID $C25_PID $C42_PID $C31_PID >> $PKILL
