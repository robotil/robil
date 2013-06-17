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

echo "Robot stops at Gate 2:"

rosservice call C25/BDIswitch "state:
   data: 0"

rosrun C42_DynamicLocomotion WalkerNode.py DW &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C0_RobilTask task_tester WalkerNode_DW args=Terra=MUD &
C1=$!
echo $C1 >> $PKILL
sleep 1
rosrun C42_DynamicLocomotion scripts/C31_Clone_VRC2_Mud_Pit_C25zero.py &
C1=$!
echo $C1 >> $PKILL
sleep 1

#echo "Robot Stops at Gate 3:"
#
#rosrun C42_DynamicLocomotion WalkerNode.py DW
#rosrun C0_RobilTask task_tester WalkerNode_DW args=Terra=HILLS
#rosrun C42_DynamicLocomotion scripts/C31_Clone_VRC2_Gate3to4_new.py

#echo "Robot Stops at Gate 4:"
#rosrun C42_DynamicLocomotion WalkerNode.py DW
#rosrun C0_RobilTask task_tester WalkerNode_DW args=Terra=HILLS
#rosrun C42_DynamicLocomotion scripts/C31_Clone_VRC2_Gate4to5_new.py
#echo $C1 $C2 $C3 $C4 $C5 >> $PKILL
