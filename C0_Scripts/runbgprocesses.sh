#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.killme.txt'
rm $PKILL
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
echo "Starting C31"
rosrun C31_PathPlanner gpp&
C31_PID=$!
echo $C31_PID
echo "Done "
echo $C22_PID $C24_PID $C25_PID $C31_PID>> $PKILL
#rosrun C21_VisionAndLidar c21 _left:=/wide_stereo/left/image_raw _right:=/wide_stereo/right/image_raw
