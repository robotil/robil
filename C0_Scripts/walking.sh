#!/bin/sh
#This script should run all the modules needed for walking task

CURR_WD=$PWD
PKILL=$CURR_WD'/.killWalking.txt'
rm $PKILL
#rosrun C42_DynamicLocomotion DynamicLocomotion.py &
rosrun C42_DynamicLocomotion WalkerNode.py BDI &
C1=$!
sleep 1
rosrun C42_DynamicLocomotion WalkerNode.py DD &
C2=$!
sleep 1
rosrun C42_DynamicLocomotion WalkerNode.py QS &
C3=$!
sleep 1
rosrun C43_LocalBodyCOM computeCOM &
C4=$!
sleep 1
rosrun C42_DynamicLocomotion WalkerNode.py DW &
C5=$!

#echo "ZMP"
#roslaunch C42_ZMPWalk zmp_walk.launch &
#C4=$!
echo $C1 $C2 $C3 $C4 $C5 >> $PKILL

