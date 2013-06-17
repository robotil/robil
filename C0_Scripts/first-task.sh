#!/bin/sh
#This script should run the whole itinerary of task_2

CURR_WD=$PWD
PKILL=$CURR_WD'/.killWalking.txt'
rm $PKILL

roslaunch C21_VisionAndLidar C21.launch &
C1=$!
echo $C1 >> $PKILL
sleep 1
roslaunch C25_GlobalPosition C25.launch &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C42_State state &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C43_LocalBodyCOM computeCOM &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C42_DynamicLocomotion manipulate_msg.py &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C42_DynamicLocomotion WalkerNode.py CD &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C0_RobilTask task_tester WalkerNode_Continuous &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C42_DynamicLocomotion scripts/C31_Clone_VRC2_StartingPentoGate2.py &
C1=$!
echo $C1 >> $PKILL
sleep 1

