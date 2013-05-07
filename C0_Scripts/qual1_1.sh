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
echo "Starting C24"
rosrun C24_ObstacleDetection           C24 &
C24_PID=$!
echo $C24_PID
sleep 3
rosrun C31_PathPlanner gpp > gpp.log &
C31_PID=$!
echo $C31_PID
echo "Starting Zmp"
roslaunch C42_ZMPWalk zmp_walk.launch &
C42_PID=$!
echo "Starting DRCSim tools"
rosrun C42_DRCSim2_tools foot_contact_filter.py &
C42T_PID=$!
echo "Starting C45"
roslaunch C45_PostureControl C45_PostureControl.launch &
C45_PID=$!
echo "Starting computeCOM"
rosrun C43_LocalBodyCOM computeCOM &
C43_PID=$!
echo "Starting QuasiStatic"
roslaunch C41_QuasiStaticWalking C41_QuasiStaticWalking.launch &
C41_PID=$!
echo $C24_PID $C25_PID $C42_PID $C42T_PID $C45_PID $C41_PID $C43_PID>> $PKILL
