#!/bin/sh
#This script should run all the modules needed for walking task

CURR_WD=$PWD
PKILL=$CURR_WD'/.killWalking.txt'
rm $PKILL
rosrun C42_DynamicLocomotion DynamicLocomotion.py &
C1=$!
sleep 1
#echo "ZMP"
#roslaunch C42_ZMPWalk zmp_walk.launch &
#C4=$!
echo $C1  >> $PKILL

