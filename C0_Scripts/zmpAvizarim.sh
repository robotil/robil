#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killZmpAvizarim.txt'
rm $PKILL
echo "This script run some tasks needed to check walk task"
echo "FallingMonitor"
rosrun C35_Monitoring FallingMonitor.py &
C1=$!
sleep 1
echo "TimeoutMonitor"
python $ROBIL/C35_Monitoring/src/TimeoutMonitor.py &
C2=$!
sleep 1
echo "Standup"
rosrun C48_StandUp C48_StandUp.py &
C3=$!
sleep 1
echo "ZMP"
roslaunch C42_ZMPWalk zmp_walk.launch &
C4=$!
echo $C1 $C2 $C3 $C4 >> $PKILL

