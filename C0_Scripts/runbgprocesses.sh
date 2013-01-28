#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.killme.txt'
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
rosrun C25_GlobalPosition              C25 &
C25_PID=$!
echo $C25_PID
echo "Starting executer"
rosrun C34_Executer executer &
C34_PID=$!
echo $C34_PID
echo "Done "
sleep 3
echo "Starting designer"
roslaunch C34_Designer start.launch &
C34D_PID=$!
echo $C34D_PID
sleep 3
echo "Done "
echo "Starting C31"
rosrun C31_PathPlanner gpp&
C31_PID=$!
echo $C31_PID
echo "Done "
echo $C21_PID $C22_PID $C24_PID $C25_PID $C31_PID $C34_PID $C34D_PID>> $PKILL
